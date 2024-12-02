#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure representing a circular buffer.
 */
typedef struct {
    volatile uint8_t* data;
    uint16_t head;
    uint16_t tail;
    uint16_t capacity;
} RingBuffer;

/**
 * @brief Initializes the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param src Pointer to the data array to be used by the buffer.
 * @param len Size of the data array.
 */
void RingBuffer_Init(RingBuffer* ringBuff, volatile uint8_t* src, uint16_t len);

/**
 * @brief Checks if the circular buffer is empty.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @return true if the buffer is empty, false otherwise.
 */
bool RingBuffer_IsEmpty(const RingBuffer* ringBuff);

/**
 * @brief Checks if the circular buffer has data.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @return true if the buffer has data, false otherwise.
 */
bool RingBuffer_HasData(const RingBuffer* ringBuff);

/**
 * @brief Returns the number of elements in the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @return Number of elements in the buffer.
 */
int RingBuffer_Count(const RingBuffer* ringBuff);

/**
 * @brief Pushes a single byte of data into the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param data Byte of data to be pushed into the buffer.
 * @return true if the data was successfully pushed, false otherwise.
 */
bool RingBuffer_Push(RingBuffer* ringBuff, uint8_t data);

/**
 * @brief Pushes an array of data into the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param src Pointer to the source data array.
 * @param len Length of the source data array.
 * @return Number of bytes successfully pushed into the buffer.
 */
uint16_t RingBuffer_PushArray(RingBuffer* ringBuff, const uint8_t* src, uint16_t len);

/**
 * @brief Peeks a single byte of data from the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param dst Pointer to the destination byte where the peeked data will be stored.
 * @return true if there was in the buffer to peek, false otherwise.
 */
bool RingBuffer_Peek(const RingBuffer* ringBuff, uint8_t* dst);

/**
 * @brief Removes a single byte of data from the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 */
void RingBuffer_Skip(RingBuffer* ringBuff);

/**
 * @brief Pops a single byte of data from the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param dst Pointer to the destination byte where the popped data will be stored.
 * @return true if the data was successfully popped, false otherwise.
 */
bool RingBuffer_Pop(RingBuffer* ringBuff, uint8_t* dst);

/**
 * @brief Pops an array of data from the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param dst Pointer to the destination data array.
 * @param len Length of the destination data array.
 * @return Number of bytes successfully popped from the buffer.
 */
uint16_t RingBuffer_PopArray(RingBuffer* ringBuff, uint8_t* dst, uint16_t len);

/**
 * @brief Clears the circular buffer.
 *
 * @param buffer Pointer to the circular buffer structure.
 */
void RingBuffer_Clear(RingBuffer* buffer);

/**
 * @brief Reads an int from the circular buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param result Pointer to the destination int where the read data will be stored.
 * @return true if the data was successfully read, false otherwise.
 */
bool RingBuffer_ReadInt(RingBuffer* ringBuff, int* result);

/**
 * @brief Reads a float from the circuilar buffer.
 *
 * @param ringBuff Pointer to the circular buffer structure.
 * @param result Pointer to the destination float where the read data will be stored.
 * @return true if the data was successfully read, false otherwise.
 */
bool RingBuffer_ReadFloat(RingBuffer* ringBuff, float* result);

#ifdef __cplusplus
}
#endif
#endif
