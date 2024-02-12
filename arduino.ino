#include <stdint.h>
typedef uint64_t u64;
typedef int8_t   i8;
#include "common/common.h"
#include "ShiftRegisterPWM.h"

#define MSG_BUFFER_CAP 128
#define KEYS_AMOUNT 88 // Amount of keys on the piano
#define STARTING_KEY PIANO_KEY_A
#define FULL_OCTAVES_AMOUNT ((88 - (PIANO_KEY_AMOUNT - STARTING_KEY))/PIANO_KEY_AMOUNT)
#define LAST_OCTAVE_LEN (KEYS_AMOUNT - (FULL_OCTAVES_AMOUNT*PIANO_KEY_AMOUNT + (PIANO_KEY_AMOUNT - STARTING_KEY)))
#define MID_OCTAVE_START_IDX ((PIANO_KEY_AMOUNT - STARTING_KEY) + PIANO_KEY_AMOUNT*(FULL_OCTAVES_AMOUNT/2))
#define MAX_KEYS_AT_ONCE 10 // The maximum amount of keys to play at once
#define MIN_KEY_VAL 185 // Minimum value to set for a motor to move, if a key should be played
#define MAX_KEY_VAL 255 // Maximum value to set for a motor to move, if a key should be played
#define CLOCK_CYCLE_LEN 88 // in milliseconds
#define SHIFT_REGISTER_COUNT 1

ShiftRegisterPWM sr(SHIFT_REGISTER_COUNT, 16);

u8 keyValues[KEYS_AMOUNT] = {0}; // This array is adressing the motors for each key on the piano
MusicChunk musicChunks[MAX_CHUNKS_AMOUNT] = {
    { .time = 1, .len = 90, .key = PIANO_KEY_A, .octave = 0, .on = true },
    { .time = 2, .len = 90, .key = PIANO_KEY_A, .octave = 0, .on = false },
};
u32 musicChunksLen  = 2;
u32 chunkIdx        = 0;
u64 musicTime       = 0;
u16 restTime        = 0;
u8  activeKeysCount = 0;
bool isMusicPlaying = true;

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
    pinMode(LED_BUILTIN, OUTPUT); // @Cleanup for debugging only

    pinMode(2, OUTPUT); // sr data pin
    pinMode(3, OUTPUT); // sr clock pin
    pinMode(4, OUTPUT); // sr ST_CP pin
    sr.interrupt(ShiftRegisterPWM::UpdateFrequency::SuperFast);

    // initialize serial port
    Serial.begin(BAUD_RATE);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.print("\n");
}

void printKeyVals()
{
    Serial.print("[");
    Serial.print(keyValues[0], DEC);
    for (u8 i = 1; i < KEYS_AMOUNT; i++) {
        Serial.print(", ");
        Serial.print(keyValues[i]);
    }
    Serial.print("]\n");
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
    Serial.flush();
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
    // Send song data to Shift registers
    for (u32 i = 0; i < KEYS_AMOUNT; i++) {
        sr.set(i, keyValues[i]);
    }
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
        }
        // Enters this block only if a new message is starting to be read
        if (!remainingLen) {
            u32 bufferLen = msgBuffer.len - msgBuffer.idx;
            bool correctMagic = false;
            while (bufferLen >= 12 && !correctMagic) {
                correctMagic = ail_buf_read4msb(&msgBuffer) == SPPP_MAGIC;
                clearPrevMsg();
            }
            if (bufferLen >= 8 && correctMagic) {
                msgType      = ail_buf_read4msb(&msgBuffer);
                remainingLen = ail_buf_read4lsb(&msgBuffer);
                if (msgType == MSG_PIDI) setPlaying(false); // Stop playing while reading the next song
            }
        }
        // Enters this block only if a message has been started to be read
        if (remainingLen) {
            switch (msgType) {
                case MSG_PIDI: {
                    u32 n = (msgBuffer.len - msgBuffer.idx)/ENCODED_MUSIC_CHUNK_LEN;
                    for (u32 i = 0; i < n; i++) {
                        musicChunks[musicChunksLen++] = decode_chunk(&msgBuffer);
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
                for (u32 i = 0; i < musicChunksLen && musicChunks[i].time <= jumpTime; i++) {
                    print_chunk(musicChunks[i]);
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
        if (chunkIdx >= musicChunksLen) {
            // to play in a loop: chunkIdx = 0;
            // otherwise setting isMusicPlaying = false; might be worthwile
            isMusicPlaying = false;
            clearMotors();
            goto done;
        }

        // Update keyValues array
        while (chunkIdx < musicChunksLen && musicChunks[chunkIdx].time <= musicTime) {
            AIL_STATIC_ASSERT(KEYS_AMOUNT <= INT8_MAX);
            MusicChunk chunk = musicChunks[chunkIdx];
            int key = MID_OCTAVE_START_IDX + PIANO_KEY_AMOUNT*(int)chunk.octave + (int)chunk.key;
            // Serial.print("Key: ");
            // Serial.println(key);
            if (key < 0) key = (chunk.key < STARTING_KEY)*(PIANO_KEY_AMOUNT) + chunk.key - STARTING_KEY;
            else key = KEYS_AMOUNT + chunk.key - LAST_OCTAVE_LEN - (chunk.key >= LAST_OCTAVE_LEN)*PIANO_KEY_AMOUNT;
            Serial.print("Key: "); // @Bug
            Serial.println(key);
            Serial.println(KEYS_AMOUNT + chunk.key - LAST_OCTAVE_LEN - (chunk.key >= LAST_OCTAVE_LEN)*PIANO_KEY_AMOUNT);

            if (activeKeysCount < MAX_KEYS_AT_ONCE) {
                if      ( chunk.on && !keyValues[key]) activeKeysCount++;
                else if (!chunk.on &&  keyValues[key]) activeKeysCount--;
                keyValues[key] = chunk.on*AIL_LERP(chunk.len/MAX_VELOCITY, MIN_KEY_VAL, MAX_KEY_VAL);
            }
            
            printKeyVals();
            chunkIdx++;
        }

        // @TODO: Adress motors using keyValues array
        // Motor on PIN 5
        // LED on PIN 2
        // sr.set(1, 0);
        // sr.set(0, 0);
        for (u8 i = 0; i < 8; i++) sr.set(i, 185);

      done:
        u64 elapsed = millis() - start;
        musicTime += elapsed/CLOCK_CYCLE_LEN;
        restTime  += elapsed%CLOCK_CYCLE_LEN;
        if (restTime >= CLOCK_CYCLE_LEN) {
            musicTime++;
            restTime -= CLOCK_CYCLE_LEN;
        }
    }
}
