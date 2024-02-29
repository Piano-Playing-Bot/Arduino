void play_note(MusicChunk *chunks, u32 *len, u64 time, u8 key, i8 octave, u8 velocity);
void stop_note(MusicChunk *chunks, u32 *len, u64 time, u8 key, i8 octave);
void sort(MusicChunk *chunks, u32 len);

u32 set_music_chunks(MusicChunk *chunks)
{
    u32 len = 0;
    
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
    play_note(chunks, &len, 0, PIANO_KEY_A, -16, 255);
    stop_note(chunks, &len, 500, PIANO_KEY_A, -16);

    sort(chunks, len);
	  return len;
}

void play_note(MusicChunk *chunks, u32 *len, u64 time, u8 key, i8 octave, u8 velocity) {
    chunks[*len].time = time;
    chunks[*len].velocity = velocity;
    chunks[*len].key = key;
    chunks[*len].octave = octave;
    chunks[*len].on = true;
    *len += 1;
}

void stop_note(MusicChunk *chunks, u32 *len, u64 time, u8 key, i8 octave) {
    chunks[*len].time = time;
    chunks[*len].velocity = 0;
    chunks[*len].key = key;
    chunks[*len].octave = octave;
    chunks[*len].on = false;
    *len += 1;
}

void sort(MusicChunk *chunks, u32 len) {
    for (u32 i = 0; i < len - 1; i++) {
        u32 min = i;
        for (u32 j = i + 1; j < len; j++) {
            if (chunks[j].time < chunks[min].time) min = j;
        }
        MusicChunk tmp = chunks[i];
        chunks[i] = chunks[min];
        chunks[min] = tmp;
    }
}