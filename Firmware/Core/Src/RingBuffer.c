#include "RingBuffer.h"

void RingBuffer_Init(RingBuffer* ringBuff, volatile uint8_t* src, const uint16_t len) {
    ringBuff->data = src;
    ringBuff->head = 0;
    ringBuff->tail = 0;
    ringBuff->capacity = len;
}

bool RingBuffer_IsEmpty(const RingBuffer* ringBuff) {
    return ringBuff->head == ringBuff->tail;
}

bool RingBuffer_HasData(const RingBuffer* ringBuff) {
    return ringBuff->head != ringBuff->tail;
}

int RingBuffer_Count(const RingBuffer* ringBuff) {
    if (ringBuff->tail == ringBuff->head)
        return 0;
    if (ringBuff->tail > ringBuff->head)
        return ringBuff->tail - ringBuff->head;
    return ringBuff->capacity - ringBuff->head + ringBuff->tail;
}

bool RingBuffer_Push(RingBuffer* ringBuff, const uint8_t data) {
    // `next` is where tail will point to after the push
    uint16_t next = ringBuff->tail + 1;
    if (next >= ringBuff->capacity)
        next = 0;
    // If tail + 1 == head, the circular buffer is full
    if (next == ringBuff->head)
        return false;
    ringBuff->data[ringBuff->tail] = data;
    ringBuff->tail = next;
    return true;
}

uint16_t RingBuffer_PushArray(RingBuffer* ringBuff, const uint8_t* src, const uint16_t len) {
    uint16_t pushed = 0;
    while (pushed < len) {
        // `next` is where tail will point to after the push
        uint16_t next = ringBuff->tail + 1;
        if (next >= ringBuff->capacity)
            next = 0;
        // If tail + 1 == head, the circular buffer is full
        if (next == ringBuff->head)
            return pushed;
        ringBuff->data[ringBuff->tail] = src[pushed++];
        ringBuff->tail = next;
    }
    return pushed;
}

bool RingBuffer_Peek(const RingBuffer* ringBuff, uint8_t* dst) {
    // If head == tail, there's no data
    if (ringBuff->tail == ringBuff->head)
        return false;
    *dst = ringBuff->data[ringBuff->head];
    return true;
}

void RingBuffer_Skip(RingBuffer* ringBuff) {
    if (ringBuff->tail != ringBuff->head) {
        uint16_t next = ringBuff->head + 1;
        if (next >= ringBuff->capacity)
            next = 0;
        ringBuff->head = next;
    }
}

bool RingBuffer_Pop(RingBuffer* ringBuff, uint8_t* dst) {
    // If head == tail, there's no data
    if (ringBuff->tail == ringBuff->head)
        return false;
    // `next` is where head will point to after the pop
    uint16_t next = ringBuff->head + 1;
    if (next >= ringBuff->capacity)
        next = 0;
    *dst = ringBuff->data[ringBuff->head];
    ringBuff->head = next;
    return true;
}

uint16_t RingBuffer_PopArray(RingBuffer* ringBuff, uint8_t* dst, const uint16_t len) {
    uint16_t popped = 0;
    while (popped < len) {
        // If head == tail, the buffer is empty
        if (ringBuff->tail == ringBuff->head)
            return popped;
        // `next` is where head will point to after the pop
        uint16_t next = ringBuff->head + 1;
        if (next >= ringBuff->capacity)
            next = 0;
        dst[popped++] = ringBuff->data[ringBuff->head];
        ringBuff->head = next;
    }
    return popped;
}

void RingBuffer_Clear(RingBuffer* buffer) {
    buffer->head = 0;
    buffer->tail = 0;
}

bool RingBuffer_ReadInt(RingBuffer* ringBuff, int* result) {
    char c;
    if (!RingBuffer_Peek(ringBuff, (uint8_t*) &c))
        return false;
    const bool isNegative = (c == '-');
    if (isNegative) RingBuffer_Skip(ringBuff);
    bool found = false;
    int32_t num = 0;
    while (RingBuffer_Pop(ringBuff, (uint8_t*) &c)) {
        if ((c < '0') || (c > '9'))
            break;
        num = num * 10 + (c - '0');
        found = true;
    }
    if (!found) return false;
    *result = isNegative ? (-num) : num;
    return true;
}

bool RingBuffer_ReadFloat(RingBuffer* ringBuff, float* result) {
    char c;
    if (!RingBuffer_Peek(ringBuff, (uint8_t*) &c))
        return false;
    const bool isNegative = (c == '-');
    if (isNegative) RingBuffer_Skip(ringBuff);
    bool found = false;
    float num = 0;
    while (RingBuffer_Pop(ringBuff, (uint8_t*) &c)) {
        if ((c < '0') || (c > '9'))
            break;
        num = num * 10.0f + ((float) (c - '0'));
        found = true;
    }
    uint32_t e = 0;
    if (c == '.') {
        while (RingBuffer_Pop(ringBuff, (uint8_t*) &c)) {
            if ((c < '0') || (c > '9'))
                break;
            num = num * 10.0f + ((float) (c - '0'));
            e++;
            found = true;
        }
    }
    if (!found) return false;
    while (e--)
        num /= 10.0f;
    *result = isNegative ? (-num) : num;
    return true;
}
