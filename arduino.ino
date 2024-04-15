#define CONST_VAR PROGMEM
#define AIL_RING_SIZE 64
#define AIL_RING_DEF static inline
#define AIL_DBG_PRINT(x) Serial.print(x)

// #define DEBUG             // uncomment to enable debug output
// #define DEBUG_SINGLE_LED  // uncomment to light up builtin-LED for specified key
#define DEBUG_CONN 0
#ifdef DEBUG
    #define PRINT(...)   Serial.print(__VA_ARGS__)
    #define PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    #define PRINT(...)   do {} while(0)
    #define PRINTLN(...) do {} while(0)
#endif

#ifdef DEBUG
    #define AIL_ASSERT_COMMON(expr, msg) do { if (!(expr)) { AIL_DBG_PRINT("Assertion failed in " __FILE__ ":" AIL_STR_LINE "\n  " msg); } } while(0)
#else
    #define AIL_ASSERT_COMMON(expr, msg) ((void)(expr));
#endif

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

// @Performance: Instead of using the ShitRegisterPWM library, we can copy the relevant parts of it and thus hardcode them
// @Performance: General optimizations can be made, like loop-unrolling, branchless-programming, etc.

// Naming Conventions
// `piano` is the array of 88 values that represents the PWM value that should be sent to each key on the piano respectively (currently called `piano`)
// A `command` is a single struct saying after how many ms a piano key should start/end playing (currently called `MusicChunk`)
// A `message` is a message as defined by the SPPProtocol - see its specification in common/ for more infos
// 'cur' == 'current'
// 'cmd' == 'command'
// 'msg' == 'message'
// 'buf' == 'buffer'

#define SHIFT_REGISTER_COUNT 11  // Amount of Shift-Registers used
#define PWM_RESOLUTION 8         // Amount of bits to use for each PWM value -> specifies maximum value for PWM values and required amount of clock cycles to address all keys on the piano
#define MIN_KEY_VAL 210          // Minimum value to set for a motor to move, if a key should be played
#define MAX_KEY_VAL 255          // Maximum value to set for a motor to move, if a key should be played
#define MIN_PERIOD_MS 40         // Minimum period that a key needs to be off for, before it can be pressed again to register two key-presses
#define KEYS_AMOUNT 88           // Amount of keys on the piano
#define STARTING_KEY PIANO_KEY_A // Lowest key on the piano we use
#define MAX_KEYS_AT_ONCE 10      // The maximum amount of keys to play at once
#define FULL_OCTAVES_AMOUNT ((88 - (PIANO_KEY_AMOUNT - STARTING_KEY))/PIANO_KEY_AMOUNT) // Amount of full octaves (containing all 12 keys) on our piano
#define LAST_OCTAVE_LEN (KEYS_AMOUNT - (FULL_OCTAVES_AMOUNT*PIANO_KEY_AMOUNT + (PIANO_KEY_AMOUNT - STARTING_KEY))) // Amount of keys in the highest (none-full) octave
#define MID_OCTAVE_START_IDX ((PIANO_KEY_AMOUNT - STARTING_KEY) + PIANO_KEY_AMOUNT*(FULL_OCTAVES_AMOUNT/2)) // Number of keys before the frst key in the middle octave on our piano
#define CMDS_LIST_LEN (32 / sizeof(PidiCmd)) // Amount of PidiCmds per buffer

typedef struct PlayedKey {
    u8  len;  // time in ms*LEN_FACTOR for which the note should still be played
    u8  idx;  // index of the note in the `piano` array.
} PlayedKey;
AIL_STATIC_ASSERT(KEYS_AMOUNT < UINT8_MAX);

typedef struct PlayedKeyList {
    u32 start_time; // time in ms to which counting the all PlayedKeys lengths is relative to
    u8  count;      // amount of currently played keys
    PlayedKey keys[MAX_KEYS_AT_ONCE];
} PlayedKeyList;
AIL_STATIC_ASSERT(MAX_KEYS_AT_ONCE < UINT8_MAX);

typedef struct {
    u16 cmds_count; // Amount of remaining commands in message
    u8  parts_read; // Amount of parts read of the PIDI message
    u8  pks_count;  // Amount of remaining played-keys in message
    PlayedKeyList new_pks;
} MsgData;

ShiftRegisterPWM sr(SHIFT_REGISTER_COUNT, PWM_RESOLUTION);

u8     piano[KEYS_AMOUNT] = { 0 }; // This array is adressing the motors for each key on the piano
u8 new_piano[KEYS_AMOUNT] = { 0 }; // The new piano to be filled while reading a NEW_MUSIC message

PidiCmd cmds_buf1[CMDS_LIST_LEN] = { 0 };
PidiCmd cmds_buf2[CMDS_LIST_LEN] = { 0 };
PlayedKeyList played_keys        = { 0 };
PidiCmd *cur_cmds       = cmds_buf1;
PidiCmd *next_cmds      = cmds_buf2;
u8   cur_cmds_count     = 0;
u8   next_cmds_count    = 0;
u8   cmd_idx            = 0;
u8   cmds_idx2          = 0;
AIL_STATIC_ASSERT(UINT8_MAX >= CMDS_LIST_LEN);
i8   pk_idx             = 0;
u8   active_keys_count  = 0;
AIL_STATIC_ASSERT(INT8_MAX >= MAX_KEYS_AT_ONCE);
u32  music_timer        = 0;
u32  prev_cmd_time      = 0;
bool is_music_playing   = true;
bool request_next_chunk = false;
AIL_RingBuffer rb       = { 0 };

f32 volume_factor = 1.0f;
f32 speed_factor  = 1.0f;

ClientMsgType msg_type = CMSG_NONE; // Type of currently parsed message
bool is_reading_msg    = false;     // whether we're currently parsing a message
MsgData msg_data       = { 0 };     // Data provided in the message (aside from the actual list of commands)

// Logically local variables, put in global scope, so that the Arduino IDE can correctly report used memory
u32  msg_start_time; // For message timeout
u32  start;
u32  i;
u32  toRead;
u8   recv_byte;
u32  buffer_size;
bool correct_magic;
u32  n;
u32  res;
u16  print_idx;
u8   encoded_cmd[ENCODED_CMD_LEN];
u8   min_idx;
u8   u8_idx;
PlayedKeySPPP msg_pidi_pk;
PlayedKey tmp_msg_pk;
const char *key_strs[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

// @Cleanup: Only required for debugging whether the values get set correctly in the piano
static inline void print_piano()
{
    PRINT(F("["));
    PRINT(piano[0], DEC);
    for (print_idx = 1; print_idx < KEYS_AMOUNT; print_idx++) {
        PRINT(F(", "));
        PRINT(piano[print_idx]);
    }
    PRINT(F("]\n"));
}

static inline void print_single_cmd(PidiCmd c)
{
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

static inline void print_cmds()
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

static inline void ring_get_from_serial(AIL_RingBuffer *rb, u32 toRead)
{
    while (toRead > 0) {
        recv_byte = Serial.read();

        // PRINT(recv_byte, HEX);
        // PRINT(F(" "));

        rb->data[rb->end] = recv_byte;
        rb->end = (rb->end + 1)%AIL_RING_SIZE;
        toRead--;
    }
}

// Send a message back to the client
static inline void send_msg(ServerMsgType type)
{
    // @Performance: Further optimization by manually inlining the Serial.write call maybe...
    PRINT(F("\n"));
    Serial.write('S');
    Serial.write('P');
    Serial.write('P');
    Serial.write(type);
    if (type == SMSG_PONG) {
        Serial.write((u8)( CMDS_LIST_LEN       & 0xff));
        Serial.write((u8)((CMDS_LIST_LEN >> 8) & 0xff));
    }
    Serial.flush(); // Very important for client to receive complete message
    PRINT(F("\n"));
}

static inline u8 get_piano_idx(PianoKey key, i8 octave)
{
    AIL_ASSERT(key < PIANO_KEY_AMOUNT);
    i16 idx = MID_OCTAVE_START_IDX + PIANO_KEY_AMOUNT*(i16)octave + (i16)key;
    if (idx < 0) idx = (key < STARTING_KEY)*(PIANO_KEY_AMOUNT) + key - STARTING_KEY;
    else if (idx >= KEYS_AMOUNT) idx = KEYS_AMOUNT + key - LAST_OCTAVE_LEN - (key >= LAST_OCTAVE_LEN)*PIANO_KEY_AMOUNT;
    AIL_ASSERT(idx >= 0);
    AIL_ASSERT(idx < KEYS_AMOUNT);
    AIL_STATIC_ASSERT(KEYS_AMOUNT <= UINT8_MAX);
    return (u8)idx;
}

#define ARR_UNORDERED_RM(arr, idx, len) (arr)[(idx)] = (arr)[--(len)]

static inline i8 get_played_key_index(u8 piano_idx, PlayedKeyList played_keys)
{
    for (i8 i = 0; i < played_keys.count; i++) {
        if (played_keys.keys[i].idx == piano_idx) return i;
    }
    return -1;
}

static inline void update_played_keys(u32 cur_time, u8 piano[KEYS_AMOUNT], PlayedKeyList *played_keys)
{
    u32 time_offset = cur_time - played_keys->start_time;
    AIL_ASSERT(cur_time >= played_keys->start_time);
    played_keys->start_time = cur_time;
    for (u8 i = 0; i < played_keys->count; i++) {
        if (played_keys->keys[i].len*LEN_FACTOR <= time_offset) {
            piano[played_keys->keys[i].idx] = 0;
            ARR_UNORDERED_RM(played_keys->keys, i, played_keys->count);
            i--;
            continue;
        }
        played_keys->keys[i].len -= time_offset/LEN_FACTOR;
    }
}

static inline void apply_cmd_no_keys_update(PidiCmd cmd, u8 piano[KEYS_AMOUNT], PlayedKeyList *played_keys)
{
    u8 idx     = get_piano_idx(pidi_key(cmd), pidi_octave(cmd));
    piano[idx] = pidi_velocity(cmd);

    i8 played_idx = -1;
    u8 next_off_key_idx = 0;
    for (u8 i = 0; i < played_keys->count; i++) {
        if (played_keys->keys[i].idx == idx) {
            played_idx = i;
        }
        if (played_keys->keys[i].len < played_keys->keys[next_off_key_idx].len) {
            next_off_key_idx = i;
        }
    }

    if (pidi_velocity(cmd)) {
        if (played_idx < 0) {
            if (played_keys->count == MAX_KEYS_AT_ONCE) {
                piano[played_keys->keys[next_off_key_idx].idx] = 0;
                played_idx = next_off_key_idx;
            } else {
                played_idx = played_keys->count++;
            }
            played_keys->keys[played_idx].idx = idx;
            played_keys->keys[played_idx].len = pidi_len(cmd);
        } else {
            played_keys->keys[played_idx].len = pidi_len(cmd);
        }
    } else if (played_idx >= 0) {
        ARR_UNORDERED_RM(played_keys->keys, played_idx, played_keys->count);
        piano[idx] = 0;
    }
}

static inline void apply_pidi_cmd(u32 cur_time, PidiCmd cmd, u8 piano[KEYS_AMOUNT], PlayedKeyList *played_keys)
{
    update_played_keys(cur_time, piano, played_keys);
    apply_cmd_no_keys_update(cmd, piano, played_keys);
}

static inline void apply_played_key(PlayedKeySPPP pk, u8 piano[KEYS_AMOUNT], PlayedKeyList *played_keys)
{
    PidiCmd cmd;
    cmd.dt       = 0,
    cmd.key      = sppp_pk_key(pk),
    cmd.len      = sppp_pk_len(pk),
    cmd.octave   = sppp_pk_octave(pk),
    cmd.velocity = sppp_pk_velocity(pk),
    apply_cmd_no_keys_update(cmd, piano, played_keys);
}

static inline void swap_cmd_buffers()
{
    cmd_idx = 0;
    cur_cmds_count = 0; // Clearing cur_cmds without having to memset anything
    AIL_SWAP_PORTABLE(u32, cur_cmds_count, next_cmds_count);
    AIL_SWAP_PORTABLE(PidiCmd*, cur_cmds, next_cmds);
    request_next_chunk = cur_cmds_count > 0;
#ifdef DEBUG
    PRINTLN(F("Swapped Buffers..."));
    print_cmds();
    print_piano();
#endif
}

// Set up I/O pins and interrupt for setting values in Shift-Registers
void setup()
{
    #ifdef DEBUG_SINGLE_LED
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
}

// Communicate with client, play song by updating shift-registers and following cmd buffers given by the client
void loop()
{
    // @TODO: speed_factor is not used yet
    ////////////////
    // Play song
    ////////////////
    // Get Start-Time for calculating elapsed time each iteration
    start = millis();

    // Set values for Shift-Registers
    if (is_music_playing) {
        for (i = 0; i < KEYS_AMOUNT; i++) {
            f32 tmp = volume_factor*piano[i];
            sr.set(i, tmp <= 0.001f ? 0 : AIL_LERP(AIL_MAX(tmp/((f32)MAX_VELOCITY), 1.0f), MIN_KEY_VAL, MAX_KEY_VAL));
        }
    } else {
        for (i = 0; i < KEYS_AMOUNT; i++) sr.set(i, 0);
    }
#ifdef DEBUG_SINGLE_LED
    if (is_music_playing && piano[MID_OCTAVE_START_IDX + PIANO_KEY_A]) {
        // PRINTLN(F("LED ON!"));
        digitalWrite(LED_BUILTIN, HIGH);
        print_piano();
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
            cmd_idx++;
        }

        for (cmds_idx2 = cmd_idx; cmds_idx2 < cur_cmds_count && prev_cmd_time + pidi_dt(cur_cmds[cmds_idx2]) <= music_timer + MIN_PERIOD_MS && played_keys.count > 0; cmds_idx2++) {
            pk_idx = get_played_key_index(get_piano_idx(cur_cmds[cmds_idx2].key, cur_cmds[cmds_idx2].octave), played_keys);
            piano[played_keys.keys[pk_idx].idx] = 0;
            ARR_UNORDERED_RM(played_keys.keys, pk_idx, played_keys.count);
        }

        // If cur_cmds is done (& next_cmds even has any cmds) & we are not in the middle of reading the next music message -> swap cur_cmds & next_cmds
        if (cmd_idx >= cur_cmds_count && next_cmds_count > 0 && !(is_reading_msg && (msg_type == CMSG_NEW_MUSIC || msg_type == CMSG_MUSIC))) {
            swap_cmd_buffers();
            goto apply_cur_cmds;
        }
    }
timer_update_done:
    ////////////////
    // Communication
    ////////////////

    // Serial.println(F("FENCE"));
    // Buffer bytes from Serial port
    toRead = Serial.available();
    if (toRead) {
        ring_get_from_serial(&rb, toRead);
        msg_start_time = start;
    }
    if (start - msg_start_time > MSG_TIMEOUT) {
        if (is_reading_msg) {
            PRINTLN(F("Message timeout"));
            msg_type = CMSG_NONE;
        }
        is_reading_msg = false;
    }

    // If starting to read a new message from the Client
    if (!is_reading_msg) {
        while (ail_ring_len(rb) >= 4 && (ail_ring_peek4msb(rb) & (0xffffff00)) != SPPP_MAGIC) {
            ail_ring_pop(&rb);
        }
        if (ail_ring_len(rb) >= 4) {
            ail_ring_popn(&rb, 3); // Remove Magic bytes
            msg_type = ail_ring_read(&rb);
            msg_data.parts_read = 0;
            is_reading_msg = true;
        }
    }

    // If message was started to be parsed
    if (is_reading_msg) {
        switch (msg_type) {
            case CMSG_NEW_MUSIC: {
                if (msg_data.parts_read == 0 && ail_ring_len(rb) >= 1) {
                    msg_data.pks_count = ail_ring_read(&rb);
                    msg_data.parts_read++;
                    msg_data.new_pks.start_time = 0;
                }
                while (msg_data.parts_read == 1 && msg_data.pks_count != 0 && ail_ring_len(rb) >= SPPP_PK_ENCODED_SIZE) {
                    apply_played_key(decode_played_key(&rb), new_piano, &msg_data.new_pks);
                    msg_data.pks_count--;
                }
                if (msg_data.parts_read == 1 && msg_data.pks_count == 0 && ail_ring_len(rb) >= 2) {
                    msg_data.cmds_count = ail_ring_read2lsb(&rb);
                    msg_data.parts_read++;
                }
                while (msg_data.parts_read == 2 && msg_data.cmds_count != 0 && ail_ring_len(rb) >= ENCODED_CMD_LEN) {
                    ail_ring_readn(&rb, ENCODED_CMD_LEN, encoded_cmd);
                    next_cmds[next_cmds_count++] = decode_cmd_simple(encoded_cmd);
                    msg_data.cmds_count--;
                }
                if (msg_data.parts_read == 2 && msg_data.cmds_count == 0) {
                    is_reading_msg = false;
                }
            } break;
            case CMSG_MUSIC: {
                if (msg_data.parts_read == 0 && ail_ring_len(rb) >= 2) {
                    msg_data.cmds_count = ail_ring_read2lsb(&rb);
                    msg_data.parts_read++;
                }
                while (msg_data.parts_read == 1 && msg_data.cmds_count != 0 && ail_ring_len(rb) >= ENCODED_CMD_LEN) {
                    ail_ring_readn(&rb, ENCODED_CMD_LEN, encoded_cmd);
                    next_cmds[next_cmds_count++] = decode_cmd_simple(encoded_cmd);
                    msg_data.cmds_count--;
                }
                if (msg_data.parts_read == 1 && msg_data.cmds_count == 0) {
                    is_reading_msg = false;
                }
            } break;
            case CMSG_CONTINUE: {
                if (ail_ring_len(rb) >= 1) {
                    is_music_playing = ail_ring_read(&rb);
                    is_reading_msg = false;
                }
            } break;
            case CMSG_VOLUME: {
                if (ail_ring_len(rb) >= 4) {
                    res = ail_ring_read4lsb(&rb);
                    volume_factor = *(f32 *)&res;
                    is_reading_msg = false;
                }
            } break;
            case CMSG_SPEED: {
                if (ail_ring_len(rb) >= 4) {
                    res = ail_ring_read4lsb(&rb);
                    speed_factor = *(f32 *)&res;
                    is_reading_msg = false;
                }
            } break;
            default:
                is_reading_msg = false;
        }
    }

    // If message was read completely
    if (!is_reading_msg) {
        switch (msg_type) {
            case CMSG_PING: {
                send_msg(SMSG_PONG);
            } break;
            case CMSG_NEW_MUSIC: {
                music_timer = 0;
                played_keys = msg_data.new_pks;
                swap_cmd_buffers();
                prev_cmd_time = 0;
                for (u8_idx = 0; u8_idx < KEYS_AMOUNT; u8_idx++) {
                    piano[u8_idx] = new_piano[u8_idx];
                    new_piano[u8_idx] = 0;
                }
                is_music_playing = true;
                send_msg(SMSG_SUCCESS);
            } break;
            case CMSG_MUSIC: {
                request_next_chunk = false;
                send_msg(SMSG_SUCCESS);
            } break;
            case CMSG_CONTINUE:
            case CMSG_VOLUME:
            case CMSG_SPEED: {
                send_msg(SMSG_SUCCESS);
            } break;
            case CMSG_NONE:
                break;
        }
        if (request_next_chunk && next_cmds_count == 0) {
            send_msg(SMSG_REQUEST);
            request_next_chunk = false;
        }
        msg_type = CMSG_NONE;
    }

    //////////////////
    // Elapsed Time //
    //////////////////
    if (is_music_playing) music_timer += speed_factor*(millis() - start);
}
