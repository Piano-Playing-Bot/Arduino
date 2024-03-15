static inline void play_note(PidiCmd *chunks, u8 *len, u64 time, u8 key, i8 octave, u8 velocity);
static inline void stop_note(PidiCmd *chunks, u8 *len, u64 time, u8 key, i8 octave);
static inline void sort(PidiCmd *chunks, u32 len);

u8 set_music_chunks(PidiCmd *chunks)
{
    u8 len = 0;

    // Note: Play/Stop more notes in any order here to test whatever you wanna test
    // Arguments are:
    // - chunks
    // - &len
    // - the time in milliseconds at which this command should be applied
    // - the key in an octave (Options are PIANO_KEY_C, PIANO_KEY_CS, PIANO_KEY_D, ..., PIANO_KEY_AS, PIANO_KEY_B)
    // - the octave (this can be any number between -128 and 127. 0 corresponds to the middle octave on the piano.
    //               -3 and 3 should correspond to the first full octaves on the piano.
    //               A piano does have keys in the partially covered octaves -4 and 4 though)
    // - the velocity to play this with (any number between 0 and 255) (don't set anything here in `stop_note()`)

    // for (i8 i = -4; i < -2; i++) {
    //   for (u8 j = 0; j < PIANO_KEY_AMOUNT; j++) {
    //     // Hier ist jetzt die Oktave i und der Key j
    //     play_note(chunks, &len, 0, j, i, 255);
    //   }
    // }

    // play_note(chunks, &len, 0, PIANO_KEY_A, -4, 255);
    // stop_note(chunks, &len, 1000000, PIANO_KEY_A, -4);

    sort(chunks, len);
	  return len;
}

static inline void play_note(PidiCmd *chunks, u8 *len, u64 time, u8 key, i8 octave, u8 velocity) {
    chunks[*len].time = time;
    chunks[*len].velocity = velocity;
    chunks[*len].key = key;
    chunks[*len].octave = octave;
    chunks[*len].on = true;
    *len += 1;
}

static inline void stop_note(PidiCmd *chunks, u8 *len, u64 time, u8 key, i8 octave) {
    chunks[*len].time = time;
    chunks[*len].velocity = 0;
    chunks[*len].key = key;
    chunks[*len].octave = octave;
    chunks[*len].on = false;
    *len += 1;
}

static inline void sort(PidiCmd *chunks, u32 len) {
    for (u32 i = 0; i < len - 1; i++) {
        u32 min = i;
        for (u32 j = i + 1; j < len; j++) {
            if (chunks[j].time < chunks[min].time) min = j;
        }
        PidiCmd tmp = chunks[i];
        chunks[i] = chunks[min];
        chunks[min] = tmp;
    }
}