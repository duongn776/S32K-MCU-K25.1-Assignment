/*******************************************************************************
 * @file    Circular_Queue.h
 * @brief   Fixed-size circular queue for S-Record text lines.
 * @details This module provides a lightweight ring buffer to temporarily store
 *          SREC lines (null-terminated ASCII) received over a stream interface
 *          (e.g., UART) before parsing/processing in a bootloader.
 *
 * @date    Oct 19, 2025
 * @author  nhduong
 ******************************************************************************/

#ifndef INC_CIRCULAR_QUEUE_H_
#define INC_CIRCULAR_QUEUE_H_

/*=============================================================================
 * Includes
 *===========================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*=============================================================================
 * Definitions
 *===========================================================================*/
/**
 * @brief Maximum number of lines the queue can store.
 */
#define QUEUE_MAX_SIZE        4U

/**
 * @brief Maximum length (in bytes) of a single stored line, including the
 *        terminating null character. Input longer than this is truncated.
 */
#define QUEUE_MAX_LINE_LEN    120U

/*=============================================================================
 * Type Definitions
 *===========================================================================*/
/**
 * @brief Circular queue container for SREC text lines.
 *
 * The buffer holds up to @ref QUEUE_MAX_SIZE lines, each up to
 * @ref QUEUE_MAX_LINE_LEN bytes (including the trailing '\0').
 */
typedef struct
{
    char     buffer[QUEUE_MAX_SIZE][QUEUE_MAX_LINE_LEN]; /*!< Storage for lines */
    uint8_t  front;     /*!< Index of the next element to pop */
    uint8_t  rear;      /*!< Index of the next slot to push  */
    uint8_t  count;     /*!< Current number of stored lines  */
} SrecQueue_t;

/*=============================================================================
 * API Prototypes
 *===========================================================================*/
void Queue_Init(SrecQueue_t *q);
bool Queue_Push(SrecQueue_t *q, const char *line);
bool Queue_Pop(SrecQueue_t *q, char *out_line);
bool Queue_IsEmpty(const SrecQueue_t *q);
bool Queue_IsFull(const SrecQueue_t *q);

#endif /* INC_CIRCULAR_QUEUE_H_ */

/*=============================================================================
 * End of File
 *===========================================================================*/
