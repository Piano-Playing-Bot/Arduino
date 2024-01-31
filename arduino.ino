#include <stdint.h>
typedef uint64_t u64;
typedef int8_t   i8;
#include "common/common.h"
#include "ShiftRegisterPWM.h"

#define MSG_BUFFER_CAP 128

ShiftRegisterPWM sr(1, 16);

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
AIL_DA(MusicChunk) chunks;

void setup() {
    chunks.data = NULL;
    chunks.cap  = 0;
    chunks.len  = 0;
    chunks.allocator = &ail_default_allocator;

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
    // @TODO
}

void clearMotors() {
    // @TODO
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

// @TODO: Add Timeout mechanism for incomplete messages
void loop() {
    // Reading Messages from the Client
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
                        ail_da_push(&chunks, decode_chunk(&msgBuffer));
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
                for (u32 i = 0; i < chunks.len && chunks.data[i].time <= jumpTime; i++) {
                    print_chunk(chunks.data[i]);
                    // @TODO: Apply Chunk
                }
                writeMsg(MSG_SUCC);
                break;
            default: // do nothing
                break;
        }
        msgType = MSG_NONE;
    }

    // @TODO: Logic for playing music
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
}
