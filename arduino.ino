#define CONST_VAR PROGMEM

#include <stdlib.h>
#include <stdint.h>
#define  u8   uint8_t
#define  u16  uint16_t
#define  u32  uint32_t
#define  u64  uint64_t
#define  i8   int8_t
#define  i16  int16_t
#define  i32  int32_t
#define  i64  int64_t
#define  f32  float
#define  f64  double
#include "common/common.h"
#include "ShiftRegisterPWM.h"

////////////////
// General Overview
////////////////
// After setting up the I/O pins and interrupts for adressing the shift-registers correctly in setup(), we do the following steps repeatedly in a loop:
// 1. Update the array "piano". This holds the velocity with which to play each key on the piano
    // If the song is paused, zeros are sent instead
// 2. Look through commands to update the "piano" array
    // To check whether a command should be applied, we use a timer, tracking the time in ms since the start of the song
    // To allow streaming commands (and thus circumvent the Arduino's small memory) we use two buffers (cur_cmds, next_cmds), that are swapped once the cur_cmds buffer was completely applied
    // Update internal timer for tracking elapsed ms since song start only if some music is playing and further cmds are queued up
// 3. Read messages
    // Load any received bytes into a static buffer, to prevent losing it when the I/O buffers are overwritten
    // Check if these bytes indicate the start of a SPPP message (if none has been started yet)
    // If a message has been starting to be read, continue reading bytes for it, building up the message data
    // When the message was parsed completely, react to it accordingly

// @TODO: Test code
// @TODO: REQP messages do not provide the 4 bytes for indicating the expected index atm. Either this code or the Protocol should be updated to be consistent
// @TODO: Messages should be discarded, if no new bytes were received after a given timeout, to make the protocol more robust
// @Performance: If we stay with max-speed at 1ms, we don't need a restTimer variable and simplify elapsed-time calculations
// @Performance: Instead of using the ShitRegisterPWM library, we can copy the relevant parts of it and thus hardcode them
// most importantly we could just send the values from the piano-array to the shift-registers directly without copying them into the library's internal "data" array first
// @Performance: General optimizations can be made, like loop-unrolling, branchless-programming, etc.

// Naming Conventions
// `piano` is the array of 88 values that represents the PWM value that should be sent to each key on the piano respectively (currently called `piano`)
// A `command` is a single struct saying after how many ms a piano key should start/end playing (currently called `MusicChunk`)
// A `message` is a message as defined by the SPPProtocol - see its specification in common/ for more infos
// 'cur' == 'current'
// 'cmd' == 'command'
// 'msg' == 'message'
// 'buf' == 'buffer'

// undefine for release
#define DEBUG

#define CLOCK_CYCLE_LEN 1        // specifies how many milliseconds each step between applying new commands should take
#define SHIFT_REGISTER_COUNT 11  // Amount of Shift-Registers used
#define PWM_RESOLUTION 8         // Amount of bits to use for each PWM value -> specifies maximum value for PWM values and required amount of clock cycles to address all keys on the piano
#define MSG_BUFFER_CAP 128       // Size of the msg-buffer
#define MIN_KEY_VAL 185          // Minimum value to set for a motor to move, if a key should be played
#define MAX_KEY_VAL 255          // Maximum value to set for a motor to move, if a key should be played

typedef struct {
    u32 chunk_index; // Index of the chunk as given by the message
    u32 parts_read;  // Amount of parts read of the PIDI message
} MsgData;

ShiftRegisterPWM sr(SHIFT_REGISTER_COUNT, PWM_RESOLUTION);

u8 piano[KEYS_AMOUNT] = {0}; // This array is adressing the motors for each key on the piano

PidiCmd cmds_buf1[CMDS_LIST_LEN]  = { 0 };
PidiCmd cmds_buf2[CMDS_LIST_LEN] = { 0 };
PidiCmd *cur_cmds  = cmds_buf1;
PidiCmd *next_cmds = cmds_buf2;
u32 cur_cmds_count    = 0;
u32 next_cmds_count   = 0;
u32 cmd_idx           = 0;
u64 music_timer       = 0;
u16 rest_timer        = 0;
u8  active_keys_count = 0;
bool is_music_playing = false;
bool requested_next_chunk = false;

f32 volume_factor = 1.0f;
f32 speed_factor  = 1.0f;

u8 msg_buffer_data[MSG_BUFFER_CAP] = {0};
AIL_Buffer msg_buffer = {
    .data = msg_buffer_data,
    .idx  = 0,
    .len  = 0,
    .cap  = MSG_BUFFER_CAP,
};
ClientMsgType msg_type = CMSG_NONE; // Type of currently parsed message
u32 remaining_msg_size = 0;        // in bytes
MsgData msg_data       = { 0 };    // Data provided in the message (aside from the actual list of commands)

// Logically local variables, put in global scope, so that the Arduino IDE can correctly report used memory
u64 start;
u32 i;
u64 elapsed;
u32 toRead;
u32 buffer_size;
bool correct_magic;
u32 n;
u32 res;
u8 print_piano_idx;
u8 clear_piano_idx;
u8 reply[12];


// @Cleanup: To check amount of free RAM during development
void display_freeram() {
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
  Serial.flush();
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);  
}

// @Cleanup: Only required for debugging whether the values get set correctly in the piano
void print_piano()
{
    Serial.print(F("["));
    Serial.print(piano[0], DEC);
    for (print_piano_idx = 1; print_piano_idx < KEYS_AMOUNT; print_piano_idx++) {
        Serial.print(F(", "));
        Serial.print(piano[i]);
    }
    Serial.print(F("]\n"));
}

// Memset piano array to 0
void clear_piano() {
    // Optimized to set 8 bytes at once
    // @Performance: Further optimization would manually unroll loop & inline function
    AIL_STATIC_ASSERT(KEYS_AMOUNT < UINT8_MAX);
    AIL_STATIC_ASSERT(KEYS_AMOUNT % sizeof(u64) == 0);
    for (clear_piano_idx = 0; clear_piano_idx < KEYS_AMOUNT/sizeof(u64); clear_piano_idx++) {
        ((u64 *)piano)[clear_piano_idx] = (u64)0;
    }
    // for (clear_piano_idx = 0; clear_piano_idx < KEYS_AMOUNT; clear_piano_idx++) {
    //     piano[clear_piano_idx] = 0;
    // }
}

// Send a message back to the client
static inline void send_msg(ServerMsgType type) {
    // @Performance: Function can easily be optimized by manually writing the reply bytes instead of using ail_buf.h
    // @Performance: Further optimization by manually inlining the Serial.write call maybe...
    Serial.flush();
    reply[0] = 'S';
    reply[1] = 'P';
    reply[2] = 'P';
    reply[3] = 'P';
    reply[4] = (u8)(type >> 24);
    reply[5] = (u8)(type >> 16);
    reply[6] = (u8)(type >> 8);
    reply[7] = (u8)type;
    *(u32 *)(&reply[8]) = 0;
    Serial.write(reply, 12);
    Serial.flush(); // Very important for client to receive complete message
}

static inline void swap_cmd_buffers() {
    send_msg(SMSG_REQP);
    requested_next_chunk = true;
    cmd_idx = 0;
    cur_cmds_count = 0; // Clearing cur_cmds without having to memset anything
    AIL_SWAP_PORTABLE(u32, cur_cmds_count, next_cmds_count);
    AIL_SWAP_PORTABLE(PidiCmd*, cur_cmds, next_cmds);
}

// Set up I/O pins and interrupt for setting values in Shift-Registers
void setup() {
    #ifdef DEBUG
        pinMode(LED_BUILTIN, OUTPUT); // @Cleanup: to provide an LED for debugging
    #endif
    pinMode(2, OUTPUT); // sr data pin
    pinMode(3, OUTPUT); // sr clock pin
    pinMode(4, OUTPUT); // sr ST_CP/FLip-FLop/Latch pin
    sr.interrupt(ShiftRegisterPWM::UpdateFrequency::VerySlow); // @TODO: Use custom frequency instead
    Serial.begin(BAUD_RATE); // Initialize serial port
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    #ifdef DEBUG
        Serial.print(F("\n")); // @Cleanup: Only useful when monitoring output with the Arduino IDE's Serial Monitor
    #endif
}

void clearPrevMsg() {
    msg_type = CMSG_NONE;
    if (msg_buffer.idx > msg_buffer.len) {
        msg_buffer.idx = 0;
        msg_buffer.len = 0;
        return;
    }
    msg_buffer.len -= msg_buffer.idx;
    for (i = 0; i < msg_buffer.len; i++) {
        msg_buffer.data[i] = msg_buffer.data[msg_buffer.idx + i];
    }
    msg_buffer.idx = 0;
}

// Communicate with client, play song by updating shift-registers and following cmd buffers given by the client
void loop() {
  #if 0
    ////////////////
    // Play song
    ////////////////
    // Get Start-Time for calculating elapsed time each iteration
    start = millis();
    
    // Set values for Shift-Registers
    if (is_music_playing) {
      for (i = 0; i < KEYS_AMOUNT; i++) {
          sr.set(i, AIL_LERP(piano[i]*speed_factor/MAX_VELOCITY, MIN_KEY_VAL, MAX_KEY_VAL));
      }
    } else {
      for (i = 0; i < KEYS_AMOUNT; i++) sr.set(i, 0);
    }
    #if 0
        if (is_music_playing && piano[0]) digitalWrite(LED_BUILTIN, HIGH);
        else digitalWrite(LED_BUILTIN, LOW);
    #endif

    // Look through cur_cmds List for updates to piano-array
    if (is_music_playing) {
apply_cur_cmds:
        while (cmd_idx < cur_cmds_count && cur_cmds[cmd_idx].time <= music_timer) {
            AIL_STATIC_ASSERT(KEYS_AMOUNT <= INT8_MAX);
            apply_pidi_cmd(piano, cur_cmds, cmd_idx, cur_cmds_count, &active_keys_count);
            #ifdef DEBUG
                print_piano();
            #endif
            cmd_idx++;
        }

        // If cur_cmds is done (& next_cmds even has any cmds) -> swap cur_cmds & next_cmds
        if (cmd_idx >= cur_cmds_count) {
            if (next_cmds_count > 0) {
                swap_cmd_buffers();
                goto apply_cur_cmds;
            } else {
                goto timer_update_done; // Don't want to increase music timer, if no cmds can be applied
            }
        }

        // Update timer
        elapsed = millis() - start;
        music_timer += elapsed/CLOCK_CYCLE_LEN;
        rest_timer  += elapsed%CLOCK_CYCLE_LEN;
        if (rest_timer >= CLOCK_CYCLE_LEN) {
            music_timer++;
            rest_timer -= CLOCK_CYCLE_LEN;
        }
    }
timer_update_done:
  #endif

    ////////////////
    // Communication
    ////////////////

// #define NEW_COMM
  #ifndef NEW_COMM
    {
        // Serial.println(F("Test"));
        toRead = Serial.available();
        if (toRead) {
            msg_buffer.len += Serial.readBytes(&msg_buffer.data[msg_buffer.len], toRead);
            Serial.print(F("Message Buffer length: "));
            Serial.println((u32)(msg_buffer.len - msg_buffer.idx));
            for (i = msg_buffer.idx; i < msg_buffer.len; i++) {
                Serial.print((char)msg_buffer.data[i]);
                Serial.print(F(", "));
            }
            Serial.print(F("\n"));
            Serial.flush();
        }
        // Enters this block only if a new message is starting to be read
        if (!remaining_msg_size) {
            buffer_size = msg_buffer.len - msg_buffer.idx;
            correct_magic = false;
            if (buffer_size >= 12 && !correct_magic) {
                correct_magic = ail_buf_read4msb(&msg_buffer) == SPPP_MAGIC;
                clearPrevMsg();
            }
            Serial.println(correct_magic);
            if (buffer_size >= 8 && correct_magic) {
                msg_type = ail_buf_read4msb(&msg_buffer);
                remaining_msg_size = ail_buf_read4lsb(&msg_buffer);
                if (remaining_msg_size > MAX_CLIENT_MSG_SIZE) remaining_msg_size = 0;
                // if (msg_type == CMSG_PIDI) is_music_playing = false; // Stop playing while reading the next song
            }
        }
        // Enters this block only if a message has been started to be read
        if (remaining_msg_size) {
            switch (msg_type) {
                case CMSG_PIDI: {
                    u32 n = (msg_buffer.len - msg_buffer.idx)/ENCODED_MUSIC_CHUNK_LEN;
                    for (u32 i = 0; i < n; i++) {
                        next_cmds[next_cmds_count++] = decode_cmd(&msg_buffer);
                    }
                    remaining_msg_size -= n*ENCODED_MUSIC_CHUNK_LEN;
                    AIL_ASSERT(remaining_msg_size % ENCODED_MUSIC_CHUNK_LEN == 0);
                } break;
                default:
                    remaining_msg_size = 0;
            }
        }
    }

    // Reacting to parsed Message
    if (!remaining_msg_size) {
        switch (msg_type) {
            case CMSG_PING:
                send_msg(SMSG_PONG);
                break;
            case CMSG_PIDI:
                clear_piano();
                is_music_playing = true;
                send_msg(SMSG_SUCC);
                break;
            case CMSG_STOP:
                is_music_playing = false;
                send_msg(SMSG_SUCC);
                break;
            case CMSG_CONT:
                is_music_playing = true;
                send_msg(SMSG_SUCC);
                break;
            default: // do nothing
                break;
        }
        msg_type = CMSG_NONE;
    }
  #else
    // Read into msg_buffer
    toRead = Serial.available();
    if (toRead) msg_buffer.len += Serial.readBytes(&msg_buffer.data[msg_buffer.len], toRead);
    if (toRead) {
        Serial.println(toRead);
        Serial.flush();
    }

    // If we aren't already in the middle of receiving a message -> try to parse next message
    if (!remaining_msg_size) {
        if (msg_buffer.idx > msg_buffer.len) msg_buffer.idx = 0; // To prevent overflow
        buffer_size = msg_buffer.len - msg_buffer.idx;
        correct_magic = false;
        // Serial.println(buffer_size);
        // Serial.print(F("---"));
        // Serial.flush();
        // Repeatedly check the first 4 bytes until the correct magic bytes for a SPPP message were found
        while (buffer_size >= 12 && !correct_magic) {
            correct_magic = ail_buf_read4msb(&msg_buffer) == SPPP_MAGIC;
            msg_type     = CMSG_NONE;
            msg_buffer.len -= msg_buffer.idx;
            for (i = 0; i < msg_buffer.len; i += sizeof(u32)) {
                // As a simple optimization, we copy u32s instead of u8s here
                // We copy u32s instead of u64, as we don't want to override the 4 bytes after the magic bytes
                *(u32 *)(&msg_buffer.data[i]) = *(u32 *)(&msg_buffer.data[msg_buffer.idx + i]);
            }
            msg_buffer.idx = 0;
        }
        if (buffer_size >= 8 && correct_magic) {
            msg_type           = ail_buf_read4msb(&msg_buffer);
            remaining_msg_size = ail_buf_read4lsb(&msg_buffer);
        }
    }
    // Serial.print(F("+++"));
    // display_freeram();
    // If a message was started -> Parse data bytes
    if (remaining_msg_size) {
        switch (msg_type) {
            case CMSG_PIDI: {
                digitalWrite(LED_BUILTIN, HIGH);
                // @Performance: Many if-cases that might be rather slow
                if (msg_data.parts_read == 0 && msg_buffer.len >= 4) {
                    msg_data.parts_read  = 1;
                    msg_data.chunk_index = ail_buf_read4lsb(&msg_buffer);
                    is_music_playing     = false;
                    remaining_msg_size  -= 4;
                }
                if (msg_data.parts_read == 1 && msg_data.chunk_index == 0 && msg_buffer.len >= 8) {
                    music_timer = ail_buf_read8lsb(&msg_buffer);
                    remaining_msg_size -= 8;
                }
                if (msg_data.parts_read == 2 && msg_data.chunk_index == 0 && msg_buffer.len >= KEYS_AMOUNT) {
                    for (i = 0; i < KEYS_AMOUNT; i++) piano[i] = msg_buffer.data[msg_buffer.idx + i];
                    msg_buffer.idx     += KEYS_AMOUNT;
                    msg_buffer.len     -= KEYS_AMOUNT;
                    remaining_msg_size -= KEYS_AMOUNT;
                }
                if ((msg_data.parts_read > 2 && msg_data.chunk_index == 0) || (msg_data.parts_read > 0 && msg_data.chunk_index > 0)) {
                    n = (msg_buffer.len - msg_buffer.idx)/ENCODED_MUSIC_CHUNK_LEN;
                    remaining_msg_size -= n*ENCODED_MUSIC_CHUNK_LEN;
                    if ((remaining_msg_size % ENCODED_MUSIC_CHUNK_LEN != 0) || (n + next_cmds_count > CMDS_LIST_LEN) || true) {
                        Serial.println(F("ERROR!"));
                        msg_type = CMSG_NONE;
                        remaining_msg_size = 0;
                        // Serial.print(F("remaining_msg_size: "));
                        // Serial.flush();
                        // Serial.println(remaining_msg_size);
                        // Serial.flush();
                    } else {
                        for (i = 0; i < n; i++) {
                            next_cmds[next_cmds_count++] = decode_cmd(&msg_buffer);
                        }
                    }
                    // AIL_ASSERT(remaining_msg_size % ENCODED_MUSIC_CHUNK_LEN == 0);
                }
            } break;
            case CMSG_LOUD: {
                if (msg_buffer.len >= 4) {
                    res = ail_buf_read4lsb(&msg_buffer);
                    volume_factor = *(f32 *)&res;
                    remaining_msg_size = 0;
                }
            } break;
            case CMSG_SPED: {
                if (msg_buffer.len >= 4) {
                    res = ail_buf_read4lsb(&msg_buffer);
                    speed_factor = *(f32 *)&res;
                    remaining_msg_size = 0;
                }
            } break;
            default:
                remaining_msg_size = 0;
        }
    }

    // If message was received completely -> React to message
    if (!remaining_msg_size) {
        digitalWrite(LED_BUILTIN, LOW);
        switch (msg_type) {
            case CMSG_PING:
                send_msg(SMSG_PONG);
                break;
            case CMSG_PIDI:
                // @TODO: Check that received PIDI-index was expected (either last index +1 or 0)
                requested_next_chunk = false;
                if (msg_data.chunk_index == 0) {
                    swap_cmd_buffers();
                    is_music_playing = true;
                }
                send_msg(SMSG_SUCC);
                display_freeram();
                Serial.flush();
                break;
            case CMSG_STOP:
                is_music_playing = false;
                send_msg(SMSG_SUCC);
                break;
            case CMSG_CONT:
                is_music_playing = true;
                send_msg(SMSG_SUCC);
                break;
            default: // do nothing
                break;
        }
        // u32 n = msg_buffer.len - msg_buffer.idx;
        // if (msg_buffer.len > msg_buffer.idx) n = 0;
        // if (n > 0) {
        //     Serial.println(n);
        //     Serial.println((u32)msg_buffer.idx);
        //     Serial.flush();
        // }
        // for (i = 0; i < n; i++) {
        //     msg_buffer.data[i] = msg_buffer.data[msg_buffer.idx + i];
        // }
        msg_buffer.idx = 0;
        msg_buffer.len = 0;
        msg_type = CMSG_NONE;
        if (requested_next_chunk) send_msg(SMSG_REQP);
    }
  #endif
}
