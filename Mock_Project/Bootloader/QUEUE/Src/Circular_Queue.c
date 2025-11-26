/*******************************************************************************
 * @file    Circular_Queue.c
 * @brief   Implementation of a fixed-size circular queue for SREC text lines.
 * @details Provides enqueue/dequeue operations for buffering S-Record lines
 *          received via communication interfaces before parsing.
 *
 * @date    Oct 19, 2025
 * @author  nhduong
 ******************************************************************************/

#include "Circular_Queue.h"

/**
 * @brief   Safe string copy with guaranteed null-termination.
 * @param   dest Destination buffer.
 * @param   src  Source string.
 * @param   n    Maximum number of bytes to copy (including '\0').
 */
static void my_strncpy(char *dest, const char *src, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n - 1 && src[i] != '\0'; i++)
    {
        dest[i] = src[i];
    }
    dest[i] = '\0';
}


/**
 * @brief Initialize the circular queue to an empty state.
 * @param q Pointer to queue instance.
 */
void Queue_Init(SrecQueue_t *q)
{
    q->front = 0U;
    q->rear  = 0U;
    q->count = 0U;
}

/**
 * @brief Check if the queue is empty.
 * @param q Pointer to queue instance.
 * @return true  if empty, false otherwise.
 */
boolean Queue_IsEmpty(const SrecQueue_t *q)
{
    return (q->count == 0U);
}

/**
 * @brief Check if the queue is full.
 * @param q Pointer to queue instance.
 * @return true  if full, false otherwise.
 */
boolean Queue_IsFull(const SrecQueue_t *q)
{
    return (q->count >= QUEUE_MAX_SIZE);
}

/**
 * @brief Push (enqueue) a new line into the queue.
 * @param q    Pointer to queue instance.
 * @param line Null-terminated string to enqueue.
 * @return true  if successfully enqueued.
 * @return false if queue is full or input is NULL.
 */
boolean Queue_Push(SrecQueue_t *q, const char *line)
{
    if (Queue_IsFull(q) || (line == NULL))
    {
        return false;
    }

    my_strncpy(q->buffer[q->rear], line, QUEUE_MAX_LINE_LEN);

    q->rear = (uint8_t)((q->rear + 1U) % QUEUE_MAX_SIZE);
    q->count++;

    return true;
}

/**
 * @brief Pop (dequeue) the oldest line from the queue.
 * @param q         Pointer to queue instance.
 * @param out_line  Destination buffer to receive the dequeued line.
 *                  Must be at least QUEUE_MAX_LINE_LEN bytes.
 * @return true  if a line was successfully dequeued.
 * @return false if the queue is empty or output buffer is NULL.
 */
boolean Queue_Pop(SrecQueue_t *q, char *out_line)
{
    if (Queue_IsEmpty(q) || (out_line == NULL))
    {
        return false;
    }

    my_strncpy(out_line, q->buffer[q->front], QUEUE_MAX_LINE_LEN);

    q->front = (uint8_t)((q->front + 1U) % QUEUE_MAX_SIZE);
    q->count--;

    return true;
}

/*=============================================================================
 * End of File
 *===========================================================================*/
