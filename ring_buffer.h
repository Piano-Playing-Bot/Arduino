#include "header.h"

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE 128
#endif // RING_BUFFER_SIZE

AIL_STATIC_ASSERT(RING_BUFFER_SIZE < UINT8_MAX);
typedef struct RingBuffer {
    u8 start;
    u8 end;
    u8 data[RING_BUFFER_SIZE];
} RingBuffer;

static inline u8 rb_len(RingBuffer rb)
{
    bool wrapped = rb.end < rb.start;
    return (!wrapped)*(rb.end - rb.start) + (wrapped)*(rb.end + RING_BUFFER_SIZE - rb.start);
}

// Returns the next byte or 0 if the buffer's length is 0
static inline u8 rb_peek(RingBuffer rb)
{
    return (rb.end != rb.start)*rb.data[rb.start];
}

static inline u8 rb_peek_at(RingBuffer rb, u8 offset)
{
    return (rb_len(rb) > offset)*rb.data[(rb.start + offset) % RING_BUFFER_SIZE];
}

static inline u32 rb_peek4msb(RingBuffer rb)
{
    return ((u32)rb_peek_at(rb, 0) << 24) | ((u32)rb_peek_at(rb, 1) << 16) | ((u32)rb_peek_at(rb, 2) << 8) | ((u32)rb_peek_at(rb, 3));
}

static inline u32 rb_peek4lsb(RingBuffer rb)
{
    return ((u32)rb_peek_at(rb, 3) << 24) | ((u32)rb_peek_at(rb, 2) << 16) | ((u32)rb_peek_at(rb, 1) << 8) | ((u32)rb_peek_at(rb, 0));
}

static inline u64 rb_peek8lsb(RingBuffer rb)
{
    return ((u64)rb_peek_at(rb, 7) << 7*8) | ((u64)rb_peek_at(rb, 6) << 6*8) | ((u64)rb_peek_at(rb, 5) << 5*8) | ((u64)rb_peek_at(rb, 4) << 4*8) |
           ((u64)rb_peek_at(rb, 3) << 3*8) | ((u64)rb_peek_at(rb, 2) << 2*8) | ((u64)rb_peek_at(rb, 1) << 1*8) | ((u64)rb_peek_at(rb, 0) << 0*8);
}

static inline void rb_peekn(RingBuffer rb, u8 n, u8 *buf)
{
    for (u8 i = 0, j = rb.start; i < n; i++, j = (j+1)%RING_BUFFER_SIZE) {
        // buf[i] = rb.data[j];
        // @Note: The more safe version would set the value to 0 if it's out of bounds
        buf[i] = rb.data[j]*(j < rb.end || (rb.end < rb.start && rb.start <= j));
    }
}

// Pops first element or does nothing if buffer is empty
static inline void rb_pop(RingBuffer *rb)
{
    if (rb->end != rb->start) rb->start = (rb->start + 1) % RING_BUFFER_SIZE;
}

static inline void rb_popn(RingBuffer *rb, u8 n)
{
    if (rb_len(*rb) < n) rb->start = rb->end = 0;
    else rb->start = (rb->start + n) % RING_BUFFER_SIZE;
}

static inline u32 rb_read4msb(RingBuffer *rb)
{
    u32 res = rb_peek4msb(*rb);
    rb_popn(rb, 4);
    return res;
}

static inline u32 rb_read4lsb(RingBuffer *rb)
{
    u32 res = rb_peek4lsb(*rb);
    rb_popn(rb, 4);
    return res;
}

static inline u64 rb_read8lsb(RingBuffer *rb)
{
    u64 res = rb_peek8lsb(*rb);
    rb_popn(rb, 8);
    return res;
}

static inline void rb_readn(RingBuffer *rb, u8 n, u8 *buf)
{
    rb_peekn(*rb, n, buf);
    rb_popn(rb, n);
}