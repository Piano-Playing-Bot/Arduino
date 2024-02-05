#include <stdint.h>
typedef uint64_t u64;
typedef int8_t   i8;
#include "common/common.h"
#include "ShiftRegisterPWM.h"

#define MSG_BUFFER_CAP 128
#define KEYS_AMOUNT 88 // Amount of keys on the piano
#define STARTING_KEY PIANO_KEY_A
#define FULL_OCTAVES_AMOUNT (88 - (PIANO_KEY_AMOUNT - STARTING_KEY))/PIANO_KEY_AMOUNT
#define LAST_OCTAVE_LEN KEYS_AMOUNT - (FULL_OCTAVES_AMOUNT*PIANO_KEY_AMOUNT + (PIANO_KEY_AMOUNT - STARTING_KEY))
#define MID_OCTAVE_START_IDX (PIANO_KEY_AMOUNT - STARTING_KEY) + PIANO_KEY_AMOUNT*(FULL_OCTAVES_AMOUNT/2)
#define MAX_KEYS_AT_ONCE 10 // The maximum amount of keys to play at once
#define MIN_KEY_VAL 165 // Minimum value to set for a motor to move, if a key should be played
#define MAX_KEY_VAL 255 // Maximum value to set for a motor to move, if a key should be played
#define CLOCK_CYCLE_LEN 8 // in milliseconds

ShiftRegisterPWM sr(1, 16);

u8 keyValues[KEYS_AMOUNT] = {0}; // This array is adressing the motors for each key on the piano
AIL_DA(MusicChunk) musicChunks;
u32 chunkIdx        = 0;
u64 musicTime       = 0;
u8  activeKeysCount = 0;
bool isMusicPlaying = false;

u8 msgBufferData[MSG_BUFFER_CAP] = {0};
AIL_Buffer msgBuffer = {
    .data = msgBufferData,
    .idx  = 0,
    .len  = 0,
    .cap  = MSG_BUFFER_CAP,
};
ClientMsgType msgType = MSG_NONE;
u32 remainingLen      = 0;
u32 jumpTime          = 0;

void setup() {
    musicChunks.data = NULL;
    musicChunks.cap  = 0;
    musicChunks.len  = 0;
    musicChunks.allocator = &ail_default_allocator;

    pinMode(LED_BUILTIN, OUTPUT); // @Cleanup for debugging only

    pinMode(2, OUTPUT); // sr data pin
    pinMode(3, OUTPUT); // sr clock pin
    pinMode(4, OUTPUT); // sr latch pin
    sr.interrupt(ShiftRegisterPWM::UpdateFrequency::SuperFast);

    // initialize serial port
    Serial.begin(BAUD);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
}

// @Cleanup: Only exists for debugging
void blink(u32 n) {
    return;
    for (u32 i = 0; i < n; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    delay(1000);
}

void setPlaying(bool val) {
    isMusicPlaying = val;
}

void clearMotors() {
    AIL_STATIC_ASSERT(KEYS_AMOUNT < UINT8_MAX);
    AIL_STATIC_ASSERT(KEYS_AMOUNT % sizeof(u64) == 0);
    for (u8 i = 0; i < KEYS_AMOUNT/sizeof(u64); i++) {
        ((u64 *)keyValues)[i] = (u64)0;
    }
    // for (u8 i = 0; i < KEYS_AMOUNT; i++) {
    //     keyValues[i] = 0;
    // }
}

void writeMsg(ServerMsgType type) {
    u8 reply[12];
    AIL_Buffer replyBuffer;
    replyBuffer.data = reply;
    replyBuffer.idx  = 0;
    replyBuffer.len  = 0;
    replyBuffer.cap  = 12;
    ail_buf_write4msb(&replyBuffer, SPPP_MAGIC);
    ail_buf_write4msb(&replyBuffer, type);
    ail_buf_write4lsb(&replyBuffer, 0);
    Serial.write((u8 *)reply, 12);
    blink(6);
}

void clearPrevMsg() {
    msgType = MSG_NONE;
    msgBuffer.len -= msgBuffer.idx;
    for (u32 i = 0; i < msgBuffer.len; i += sizeof(u64)) { // Basically unrolled memcpy
        *(u64 *)(&msgBuffer.data[i]) = *(u64 *)(&msgBuffer.data[msgBuffer.idx + i]);
    }
    msgBuffer.idx = 0;
}

void loop() {
    u64 start = millis();
    // Reading Messages from the Client
    // @TODO: Add Timeout mechanism for incomplete messages
    {
        u32 toRead = Serial.available();
        if (toRead) {
            msgBuffer.len += Serial.readBytes(&msgBuffer.data[msgBuffer.len], toRead);
            // Serial.print("Message Buffer length: ");
            // Serial.println((u32)(msgBuffer.len - msgBuffer.idx));
            for (u32 i = msgBuffer.idx; i < msgBuffer.len; i++) {
                // Serial.print((char)msgBuffer.data[i]);
                // Serial.print(", ");
            }
            // Serial.print("\n");
            blink(1);
        }
        // Enters this block only if a new message is starting to be read
        if (!remainingLen) {
            u32 bufferLen = msgBuffer.len - msgBuffer.idx;
            bool correctMagic = false;
            while (bufferLen >= 12 && !correctMagic) {
                correctMagic = ail_buf_read4msb(&msgBuffer) == SPPP_MAGIC;
                clearPrevMsg();
                blink(2);
            }
            if (bufferLen >= 8 && correctMagic) {
                msgType      = ail_buf_read4msb(&msgBuffer);
                remainingLen = ail_buf_read4lsb(&msgBuffer);
                if (msgType == MSG_PIDI) setPlaying(false); // Stop playing while reading the next song
                blink(3);
            }
        }
        // Enters this block only if a message had been started to be read
        if (remainingLen) {
            switch (msgType) {
                case MSG_PIDI: {
                    u32 n = (msgBuffer.len - msgBuffer.idx)/ENCODED_MUSIC_CHUNK_LEN;
                    for (u32 i = 0; i < n; i++) {
                        ail_da_push(&musicChunks, decode_chunk(&msgBuffer));
                    }
                    remainingLen -= n*ENCODED_MUSIC_CHUNK_LEN;
                    AIL_ASSERT(remainingLen % ENCODED_MUSIC_CHUNK_LEN == 0);
                } break;
                case MSG_JUMP: {
                    if (msgBuffer.len - msgBuffer.idx >= 8) {
                        jumpTime = ail_buf_read8lsb(&msgBuffer);
                        remainingLen = 0;
                    }
                } break;
                default:
                    blink(4);
                    remainingLen = 0;
            }
        }
    }

    // Reacting to parsed Message
    if (!remainingLen) {
        switch (msgType) {
            case MSG_PING:
                writeMsg(MSG_PONG);
                break;
            case MSG_PIDI:
                clearMotors();
                setPlaying(true);
                writeMsg(MSG_SUCC);
                break;
            case MSG_STOP:
                setPlaying(false);
                writeMsg(MSG_SUCC);
                break;
            case MSG_CONT:
                setPlaying(true);
                writeMsg(MSG_SUCC);
                break;
            case MSG_JUMP:
                clearMotors();
                Serial.print("Jumping to time: ");
                Serial.println(jumpTime);
                for (u32 i = 0; i < musicChunks.len && musicChunks.data[i].time <= jumpTime; i++) {
                    print_chunk(musicChunks.data[i]);
                    // @TODO: Apply Chunk
                }
                writeMsg(MSG_SUCC);
                break;
            default: // do nothing
                break;
        }
        msgType = MSG_NONE;
    }

    // Play Music
    if (isMusicPlaying) {
        // Check if music is done
        if (chunkIdx >= musicChunks.len) {
            clearMotors();
            // to play in a loop: chunkIdx = 0;
            // otherwise setting isMusicPlaying = false; might be worthwile
            break;
        }

        // Update keyValues array
        while (musicChunks.data[chunkIdx].time <= musicTime) {
            AIL_STATIC_ASSERT(KEYS_AMOUNT <= INT8_MAX);
            MusicChunk chunk = musicChunks.data[chunkIdx];
            i8 key = MID_OCTAVE_START_IDX + PIANO_KEY_AMOUNT*chunk.octave + chunk.key;
            if (key < 0) key = (chunk.key < STARTING_KEY)*(PIANO_KEY_AMOUNT - STARTING_KEY) + chunk.key;
            else key = KEYS_AMOUNT - LAST_OCTAVE_LEN - (chunk.key >= LAST_OCTAVE_LEN)*PIANO_KEY_AMOUNT + chunk.key;

            if (activeKeysCount < MAX_KEYS_AT_ONCE) {
                if      ( chunk.on && !keyValues[key]) activeKeysCount++;
                else if (!chunk.on &&  keyValues[key]) activeKeysCount--;
                keyValues[key]   = chunk.on*AIL_LERP(chunk.len/MAX_VELOCITY, MIN_KEY_VAL, MAX_KEY_VAL);
                chunkIdx++;
            }
        }

        // @TODO: Adress motors using keyValues array
        // Motor on PIN 5
        // LED on PIN 2
        // uint8_t j = 2;
        // sr.set(j, 165); //165 statt 255
        // delay(1000);

        // for (uint8_t i = 0; i < 8; i++) {
        //     sr.set(i, 0); // all off
        // }
        // delay(1000);

        // j = 5;
        // sr.set(j, 195);
        // j = 2;
        // sr.set(j, 255); // Maximal
        // delay(1000); //

        // for (uint8_t i = 0; i < 8; i++) {
        //     sr.set(i, 0);
        // }
        // delay(1000);

        u64 elapsed = millis() - start;
        if (elapsed < clock) {
            musicTime++;
            delay(clock - elapsed - 1);
        } else {
            musicTime += 1; + elapsed/CLOCK_CYCLE_LEN;
        }
    }
}
