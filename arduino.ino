#define CONST_VAR PROGMEM
#define AIL_RING_SIZE 128
#define AIL_RING_DEF static inline
#define AIL_DBG_PRINT(x) Serial.print(x)
#define AIL_ASSERT_COMMON(expr, msg) do { if (!(expr)) { AIL_DBG_PRINT("Assertion failed in " __FILE__ ":" AIL_STR_LINE "\n  " msg); } } while(0)

#include "header.h"
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

// @TODO: REQP messages do not provide the 4 bytes for indicating the expected index atm. Either this code or the Protocol should be updated to be consistent
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
#define DEBUG_CONN 0
// #define LOOPING

#define CLOCK_CYCLE_LEN 1        // specifies how many milliseconds each step between applying new commands should take
#define SHIFT_REGISTER_COUNT 11  // Amount of Shift-Registers used
#define PWM_RESOLUTION 8         // Amount of bits to use for each PWM value -> specifies maximum value for PWM values and required amount of clock cycles to address all keys on the piano
#define MIN_KEY_VAL 210          // Minimum value to set for a motor to move, if a key should be played
#define MAX_KEY_VAL 255          // Maximum value to set for a motor to move, if a key should be played

#define PRINT(...)   Serial.print(__VA_ARGS__) // do {} while(0)
#define PRINTLN(...) Serial.println(__VA_ARGS__) // do {} while(0)

typedef struct {
    u32 chunk_index; // Index of the chunk as given by the message
    u32 parts_read;  // Amount of parts read of the PIDI message
    u32 new_time;    // New time to start playing at
    u8  new_piano[KEYS_AMOUNT]; // New starting piano PWM values
} MsgData;

ShiftRegisterPWM sr(SHIFT_REGISTER_COUNT, PWM_RESOLUTION);

u8 piano[KEYS_AMOUNT] = {0}; // This array is adressing the motors for each key on the piano

PidiCmd cmds_buf1[CMDS_LIST_LEN] = { 0 };
PidiCmd cmds_buf2[CMDS_LIST_LEN] = { 0 };
PlayedKeyList played_keys        = { 0 };
PidiCmd *cur_cmds       = cmds_buf1;
PidiCmd *next_cmds      = cmds_buf2;
u32 cur_cmds_count      = 0;
u32 next_cmds_count     = 0;
u32 cmd_idx             = 0;
u32 music_timer         = 0;
u32 prev_cmd_time       = 0;
u8  active_keys_count   = 0;
bool is_music_playing   = false;
bool request_next_chunk = false;
AIL_RingBuffer rb       = {0};

f32 volume_factor = 1.0f;
f32 speed_factor  = 1.0f;

ClientMsgType msg_type = CMSG_NONE; // Type of currently parsed message
u32 remaining_msg_size = 0;        // in bytes
MsgData msg_data       = { 0 };    // Data provided in the message (aside from the actual list of commands)

// Logically local variables, put in global scope, so that the Arduino IDE can correctly report used memory
u32 msg_start_time; // For message timeout
u32 start;
u32 i;
u32 toRead;
u32 buffer_size;
bool correct_magic;
u32 n;
u32 res;
u16 print_idx;
u8 clear_piano_idx;
u8 reply[12];
u8 encoded_cmd[ENCODED_CMD_LEN];

// @Cleanup: Only required for debugging whether the values get set correctly in the piano
void print_piano()
{
    PRINT(F("["));
    PRINT(piano[0], DEC);
    for (print_idx = 1; print_idx < KEYS_AMOUNT; print_idx++) {
        PRINT(F(", "));
        PRINT(piano[print_idx]);
    }
    PRINT(F("]\n"));
}

void print_single_cmd(PidiCmd c)
{
    static const char *key_strs[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
    PRINT(F("{ key: "));
    PRINT(key_strs[pidi_key(c)]);
    PRINT(F(", octave: "));
    PRINT(pidi_octave(c));
    PRINT(F(", dt: "));
    PRINT(pidi_dt(c));
    PRINT(F("ms, len: "));
    PRINT(pidi_len(c)*LEN_FACTOR);
    PRINT(F("ms, velocity: "));
    PRINT(pidi_velocity(c));
    PRINT(F(" }"));
}

void print_cmds()
{
    PRINT(F("Commands ("));
    PRINT(cur_cmds_count);
    PRINT(F(") ["));
    if (cur_cmds_count > 0) {
        PRINT(F("\n    "));
        print_single_cmd(cur_cmds[0]);
        for (print_idx = 1; print_idx < cur_cmds_count; print_idx++) {
            PRINT(F(",\n    "));
            print_single_cmd(cur_cmds[print_idx]);
        }
    }
    PRINT(F("\n]\n"));
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

static inline void ring_get_from_serial(AIL_RingBuffer *rb, u32 toRead)
{
#if 1
    while (toRead > 0) {
        u8 x = Serial.read();
#if 1
        Serial.print(x, HEX);
        Serial.print(F(" "));
#endif
        rb->data[rb->end] = x;
        rb->end = (rb->end + 1)%AIL_RING_SIZE;
        toRead--;
    }
#else
    u8 initial_end = rb->end;
    u8 rest = AIL_RING_SIZE - rb->end;
    if (toRead < rest) {
        rb->end += Serial.readBytes(&(rb->data[rb->end]), toRead);
    } else {
        Serial.readBytes(&(rb->data[rb->end]), rest);
        rb->end = toRead - rest;
        Serial.readBytes(rb->data, rb->end);
    }
#if DEBUG_CONN
    for (u8 i = initial_end; i != rb->end; i = (i + 1)%AIL_RING_SIZE) {
        Serial.print(rb->data[i], HEX);
        Serial.print(F(" "));
    }
#endif
#endif
}

// Send a message back to the client
static inline void send_msg(ServerMsgType type) {
    // @Performance: Further optimization by manually inlining the Serial.write call maybe...
    Serial.print(F("\n"));
    // Serial.flush();
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
    Serial.print(F("\n"));
    Serial.flush(); // Very important for client to receive complete message
}

static inline void swap_cmd_buffers() {
    cmd_idx = 0;
    cur_cmds_count = 0; // Clearing cur_cmds without having to memset anything
    AIL_SWAP_PORTABLE(u32, cur_cmds_count, next_cmds_count);
    AIL_SWAP_PORTABLE(PidiCmd*, cur_cmds, next_cmds);
    request_next_chunk = cur_cmds_count > 0;
#ifdef DEBUG
    PRINTLN(F("Swapped Buffers..."));
    print_cmds();
#endif
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
        PRINT(F("\n")); // @Cleanup: Only useful when monitoring output with the Arduino IDE's Serial Monitor
    #endif

    // cur_cmds_count = set_music_chunks(cur_cmds);
    // is_music_playing = true;
}

// Communicate with client, play song by updating shift-registers and following cmd buffers given by the client
void loop() {
    // @TODO: speed_factor is not used yet
    ////////////////
    // Play song
    ////////////////
    // Get Start-Time for calculating elapsed time each iteration
    // PRINTLN(F("Test"));
    // Serial.flush();
    start = millis();

    // Set values for Shift-Registers
    if (is_music_playing) {
      for (i = 0; i < KEYS_AMOUNT; i++) {
          u8 x = piano[i];
          if (x > 0) x = MIN_KEY_VAL; // AIL_LERP(AIL_MAX(volume_factor*piano[i], MAX_VELOCITY), MIN_KEY_VAL, MAX_KEY_VAL);
          sr.set(i, x);
      }
    } else {
        for (i = 0; i < KEYS_AMOUNT; i++) sr.set(i, 0);
    }
#ifdef DEBUG
    if (is_music_playing && piano[MID_OCTAVE_START_IDX + PIANO_KEY_A]) {
        // PRINTLN(F("LED ON!"));
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }
#endif

    // Look through cur_cmds List for updates to piano-array
    if (is_music_playing) {
apply_cur_cmds:
        if (music_timer < played_keys.start_time) {
            Serial.print(F("Current Time: "));
            Serial.print(music_timer);
            Serial.print(F(", Played Keys Time: "));
            Serial.println(played_keys.start_time);
        }
        update_played_keys(music_timer, piano, &played_keys);
        while (cmd_idx < cur_cmds_count && prev_cmd_time + pidi_dt(cur_cmds[cmd_idx]) <= music_timer) {
            apply_cmd_no_keys_update(cur_cmds[cmd_idx], piano, &played_keys);
            prev_cmd_time += pidi_dt(cur_cmds[cmd_idx]);
            // Serial.println(F("Applying Command..."));
            cmd_idx++;
        }

#ifdef LOOPING
        if (cmd_idx >= cur_cmds_count) {
            // Serial.println(F("..."));
            for (u8 piano_idx = 0; piano_idx < KEYS_AMOUNT; piano_idx++) {
                piano[piano_idx] = 0;
            }
            cmd_idx = 0;
            music_timer = 0;
        }
#endif

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
        music_timer += millis() - start;
    }
timer_update_done:
    ////////////////
    // Communication
    ////////////////

    // Buffer bytes from Serial port
    toRead = Serial.available();
    if (toRead) {
        ring_get_from_serial(&rb, toRead);
        msg_start_time = start;
    }
    if (start - msg_start_time > MSG_TIMEOUT) {
        if (remaining_msg_size) {
            Serial.print(F("\nMessage timeout - Remaining Msg Size: "));
            Serial.println(remaining_msg_size);
            msg_type = CMSG_NONE;
        }
        remaining_msg_size = 0;
        rb.start = 0;
        rb.end   = 0;
    }

    // If starting to read a new message from the Client
    if (!remaining_msg_size) {
        while (ail_ring_len(rb) >= 12 && ail_ring_peek4msb(rb) != SPPP_MAGIC) {
            ail_ring_pop(&rb);
        }
        if (ail_ring_len(rb) >= 12) {
            ail_ring_popn(&rb, 4); // Magic bytes (must be correct bc of loop catching it otherwise before)
            msg_type = ail_ring_read4msb(&rb);
            msg_data.parts_read = 0;
            remaining_msg_size = ail_ring_read4lsb(&rb);
            if (remaining_msg_size > MAX_CLIENT_MSG_SIZE) remaining_msg_size = 0;
        }
    }

    // If message was started to be parsed
    if (remaining_msg_size) {
        switch (msg_type) {
            case CMSG_PIDI: {
                // Index: 4 bytes
                if (msg_data.parts_read == 0 && ail_ring_len(rb) >= 4) {
                    next_cmds_count = 0;
                    // request_next_chunk = cur_cmds_count > 0;
                    msg_data.chunk_index = ail_ring_read4lsb(&rb);
                    msg_data.parts_read++;
                    if (remaining_msg_size < 4) remaining_msg_size = 0;
                    else remaining_msg_size -= 4;
#if DEBUG_CONN
                    PRINT(F("PIDI Index: "));
                    PRINTLN(msg_data.chunk_index);
                    PRINT(F("Remaining msg size: "));
                    PRINTLN(remaining_msg_size);
#endif
                }
                // Time: 4 bytes
                if (msg_data.chunk_index == 0 && msg_data.parts_read == 1 && ail_ring_len(rb) >= 4) {
                    msg_data.new_time = ail_ring_peek8lsb(rb); ail_ring_popn(&rb, 4);
                    msg_data.parts_read++;
                    if (remaining_msg_size < 4 + KEYS_AMOUNT) remaining_msg_size = 0;
                    remaining_msg_size -= 4;
#if DEBUG_CONN
                    PRINT(F("PIDI Time: "));
                    PRINTLN((u32)msg_data.new_time);
#endif
                }
                if (msg_data.chunk_index == 0 && msg_data.parts_read == 2 && ail_ring_len(rb) >= KEYS_AMOUNT) {
                    AIL_STATIC_ASSERT(KEYS_AMOUNT < AIL_RING_SIZE);
                    ail_ring_readn(&rb, KEYS_AMOUNT, msg_data.new_piano);
                    msg_data.parts_read++;
                    remaining_msg_size -= KEYS_AMOUNT;
                    PRINTLN(F("Read piano"));
                }
                if ((msg_data.chunk_index > 0 && msg_data.parts_read > 0) || (msg_data.chunk_index == 0 && msg_data.parts_read == 3)) {
                    n = AIL_MIN(ail_ring_len(rb), remaining_msg_size)/ENCODED_CMD_LEN;
#if DEBUG_CONN
                    if (n > 0) {
                        PRINT(F("n: "));
                        PRINTLN(n);
                    }
#endif
                    for (i = 0; i < n; i++) {
                        ail_ring_readn(&rb, ENCODED_CMD_LEN, encoded_cmd);
#if DEBUG_CONN
                        for (u8 idx = 0; idx < ENCODED_CMD_LEN; idx++) { PRINT(encoded_cmd[idx], HEX); PRINT(F(" ")); }
                        PRINT(F(" -> "));
#endif
                        next_cmds[next_cmds_count++] = decode_cmd_simple(encoded_cmd);
#if DEBUG_CONN
                        print_single_cmd(next_cmds[next_cmds_count - 1]);
                        PRINT(F("\n"));
#endif
                    }
                    remaining_msg_size -= n*ENCODED_CMD_LEN - (remaining_msg_size%ENCODED_CMD_LEN); // subtract any modulo rests, to make sure we always finish reading the message
                }
            } break;
            case CMSG_LOUD: {
                if (ail_ring_len(rb) >= 4) {
                    res = ail_ring_read4lsb(&rb);
                    volume_factor = *(f32 *)&res;
                    remaining_msg_size = 0;
                }
            } break;
            case CMSG_SPED: {
                if (ail_ring_len(rb) >= 4) {
                    res = ail_ring_read4lsb(&rb);
                    speed_factor = *(f32 *)&res;
                    remaining_msg_size = 0;
                }
            } break;
            default:
                remaining_msg_size = 0;
        }
    }

    // If message was read completely
    if (!remaining_msg_size) {
        switch (msg_type) {
            case CMSG_PING: {
                send_msg(SMSG_PONG);
            } break;
            case CMSG_PIDI: {
                // @TODO: Check that received PIDI-index was expected (either last index +1 or 0)
                request_next_chunk = false;
                if (msg_data.chunk_index == 0) {
                    // @TODO: This doesn't work with played_keys system anymore
                    music_timer = msg_data.new_time;
                    memcpy(piano, msg_data.new_piano, KEYS_AMOUNT);
                    swap_cmd_buffers();
                    is_music_playing = true;
                    // print_piano();
                }
                send_msg(SMSG_SUCC);
            } break;
            case CMSG_STOP: {
                is_music_playing = false;
                send_msg(SMSG_SUCC);
            } break;
            case CMSG_CONT: {
                is_music_playing = true;
                send_msg(SMSG_SUCC);
            } break;
            case CMSG_LOUD:
            case CMSG_SPED: {
                send_msg(SMSG_SUCC);
            } break;
            case CMSG_NONE:
                break;
        }
        if (request_next_chunk && next_cmds_count == 0) {
            send_msg(SMSG_REQP);
            request_next_chunk = false;
        }
        msg_type = CMSG_NONE;
    }
}
