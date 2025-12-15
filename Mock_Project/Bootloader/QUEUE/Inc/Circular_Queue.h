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
#include "S32K144.h"
#include<string.h>

/*=============================================================================
 * Definitions
 *===========================================================================*/
#define QUEUE_MAX_SIZE        30
#define QUEUE_MAX_LINE_LEN    70
#define NULL ((void *)0)
#define true 1
#define false 0
/*=============================================================================
 * Type Definitions
 *===========================================================================*/
typedef uint8_t boolean;

typedef struct
{
    char buffer[QUEUE_MAX_SIZE][QUEUE_MAX_LINE_LEN]; /*!< Storage for lines */
    int  front;     /*!< Index of the next element to pop */
    int  rear;      /*!< Index of the next slot to push  */
    int  count;     /*!< Current number of stored lines  */
} SrecQueue_t;

/*=============================================================================
 * API Prototypes
 *===========================================================================*/
void Queue_Init(SrecQueue_t *q);
boolean Queue_Push(SrecQueue_t *q, const char *line);
boolean Queue_Pop(SrecQueue_t *q, char *out_line);
boolean Queue_IsEmpty(const SrecQueue_t *q);
boolean Queue_IsFull(const SrecQueue_t *q);

#endif /* INC_CIRCULAR_QUEUE_H_ */

/*=============================================================================
 * End of File
 *===========================================================================*/
