/*
 * HAL_USART.h
 *
 *  Created on: Oct 9, 2025
 *      Author: nhduong
 */

#ifndef INC_HAL_USART_H_
#define INC_HAL_USART_H_

#include "HAL_Common.h"
#include "core_cm4.h"

#define FREQ_USART_CLK   40000000U   // SPLL_DIV2 default

/**
  * @brief USART Init Structure definition
  */
typedef struct
{
    uint8_t     Mode;           /*!< Specifes whether the Receive or Transmit mode is enabled or disabled.
                                     This parameter can be a value of @ref USART_Mode                           */

    uint32_t    BaudRate;       /*!< This member configure the Usart communication baud rate.
                                     The parameter can be a value of @ref @USART_BaudRate                       */
    
    uint8_t     WordLength;     /*!< Specifies the number of data bits transmitted or received in a frame.
                                     The parameter can be a value of @ref USART_Word_Length                     */

    uint8_t     StopBits;       /*!< Specifes the number of stop bits transmitted.
                                     The parameter can be a value of @ref USART_Stop_Bits                       */

    uint8_t     ParityControl;  /*!< Specifies the parity control mode.
                                     This parameter can be a value of @ref USART_Parity_Control                 */

    uint8_t     HWFlowControl;  /*!< Specifies the hardware flow control mode.
                                     This parameter can be a value of @ref USART_HW_FlowControl                 */

}USART_InitTypeDef;

/**
  * @brief USART Handle Structure definition
  */
typedef struct
{
	LPUART_Type          	*pUSARTx;       /*!< Usart registers base address               */

    USART_InitTypeDef       Init;           /*!< Usart communication parameters             */

    uint8_t                 *pTxBuffer;     /*!< Pointer to Usart Tx transfer Buffer        */

    uint8_t                 *pRxBuffer;     /*!< Pointer to Usart Rx transfer Buffer        */

    uint32_t                TxLen;          /*!< Usart Tx Transfer length                   */

    uint32_t                RxLen;          /*!< Usart Rx Transfer length                   */

    uint8_t                 TxState;        /*!< Usart Tx Transfer state                    */

    uint8_t                 RxState;        /*!< Usart Rx Transfer state                    */
    void (*CallbackEvent) (uint32_t event);  /*!< Usart Callback function                    */

}USART_HandleTypeDef;

/** @defgroup USART_Mode USART Mode
  *
  */
#define USART_MODE_TX               0
#define USART_MODE_RX               1
#define USART_MODE_TX_RX            2

/** @defgroup USART_BaudRate USART BaudRate
  *
  */
#define USART_BAUDRATE_9600                 9600
#define USART_BAUDRATE_19200                19200
#define USART_BAUDRATE_38400                38400
#define USART_BAUDRATE_57600                57600
#define USART_BAUDRATE_115200               115200

/** @defgroup USART_Parity USART Parity
  *
  */
#define USART_PARITY_NONE       0
#define USART_PARITY_EVEN       1
#define USART_PARITY_ODD        2

/** @defgroup USART_Word_Length USART Word Length
  *
  */
#define USART_WORDLENGTH_8BITS      0
#define USART_WORDLENGTH_9BITS      1


/** @defgroup USART_Stop_Bits USART Number of Stop Bits
  *
  */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_2     1

/** @defgroup USART_HW_FlowControl USART Hardware Flow control
  *
  */
#define USART_HW_NONE               0
#define USART_HW_CTS                1
#define USART_HW_RTS                2
#define USART_HW_CTS_RTS            3


/** @defgroup USART_States USART States
  *
  */
#define USART_STATE_READY           0
#define USART_STATE_BUSY_TX         1
#define USART_STATE_BUSY_RX         2

/** @defgroup USART_Event_Error USART Event and Error
  *
  */
#define USART_EVENT_SEND_COMPLETE       (1UL << 0)  ///< Send completed; however USART may still transmit data
#define USART_EVENT_RECEIVE_COMPLETE    (1UL << 1)  ///< Receive completed
#define USART_EVENT_TRANSFER_COMPLETE   (1UL << 2)  ///< Transfer completed
#define USART_EVENT_TX_COMPLETE         (1UL << 3)  ///< Transmit completed (optional)
#define USART_EVENT_TX_UNDERFLOW        (1UL << 4)  ///< Transmit data not available (Synchronous Slave)
#define USART_EVENT_RX_OVERFLOW         (1UL << 5)  ///< Receive data overflow
#define USART_EVENT_RX_TIMEOUT          (1UL << 6)  ///< Receive character timeout (optional)
#define USART_EVENT_RX_BREAK            (1UL << 7)  ///< Break detected on receive
#define USART_EVENT_RX_FRAMING_ERROR    (1UL << 8)  ///< Framing error detected on receive
#define USART_EVENT_RX_PARITY_ERROR     (1UL << 9)  ///< Parity error detected on receive
#define USART_EVENT_CTS                 (1UL << 10) ///< CTS state changed (optional)
#define USART_EVENT_DSR                 (1UL << 11) ///< DSR state changed (optional)
#define USART_EVENT_DCD                 (1UL << 12) ///< DCD state changed (optional)
#define USART_EVENT_RI                  (1UL << 13) ///< RI  state changed (optional)

/******************************************************************************************
 *                              APIs supported by this driver
 *       For more information about the APIs check the function definitions
 ******************************************************************************************/

/* Peripheral and Port Clock Control */
void HAL_USART_PeriClockControl(LPUART_Type *pUSARTx, uint8_t state);
void HAL_USART_PortClockControl(LPUART_Type *pUSARTx, uint8_t state);

/* Initialization and De-initialization */
void HAL_USART_Init(USART_HandleTypeDef *husart);
void HAL_USART_DeInit(USART_HandleTypeDef *husart);

void HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxBuffer, uint32_t Len);
void HAL_USART_Receive(USART_HandleTypeDef *husart,uint8_t *pRxBuffer, uint32_t Len);

uint8_t HAL_USART_Transmit_IT(USART_HandleTypeDef *husart,uint8_t *pTxBuffer, uint32_t Len);
uint8_t HAL_USART_Receive_IT(USART_HandleTypeDef *husart,uint8_t *pRxBuffer, uint32_t Len);

void USART_SetBaudRate(LPUART_Type *pUSARTx, uint32_t BaudRate);

/* Set USART word length */
void HAL_USART_Set7BitMode(LPUART_Type *pUSARTx);
void HAL_USART_Set8BitMode(LPUART_Type *pUSARTx);
void HAL_USART_Set9BitMode(LPUART_Type *pUSARTx);

/* Set USART parity mode */
void HAL_USART_SetParityNone(LPUART_Type *pUSARTx);
void HAL_USART_SetParityEven(LPUART_Type *pUSARTx);
void HAL_USART_SetParityOdd(LPUART_Type *pUSARTx);

/* Set USART stop bits */
void HAL_USART_SetStopBits1(LPUART_Type *pUSARTx);
void HAL_USART_SetStopBits2(LPUART_Type *pUSARTx);

/* Set USART flow control */
void HAL_USART_FlowControlNone(LPUART_Type *pUSARTx);
void HAL_USART_FlowControlRTS(LPUART_Type *pUSARTx);
void HAL_USART_FlowControlCTS(LPUART_Type *pUSARTx);
void HAL_USART_FlowControlRTS_CTS(LPUART_Type *pUSARTx);

/* Enable or Disable USART transmitter and receiver */
void HAL_USART_EnableTx(USART_HandleTypeDef *husart);
void HAL_USART_EnableRx(USART_HandleTypeDef *husart);
void HAL_USART_DisableTx(USART_HandleTypeDef *husart);
void HAL_USART_DisableRx(USART_HandleTypeDef *husart);

/* Abort ongoing Tx or Rx */
void HAL_USART_AbortTx(USART_HandleTypeDef *husart);
void HAL_USART_AbortRx(USART_HandleTypeDef *husart);

/* Send or Stop a break condition on the USART */
void HAL_USART_SendBreak(USART_HandleTypeDef *husart);
void HAL_USART_StopBreak(USART_HandleTypeDef *husart);


void HAL_USART_ClearORFlag(LPUART_Type *pUSARTx);
void HAL_USART_ClearFEFlag(LPUART_Type *pUSARTx);
void HAL_USART_ClearPFFlag(LPUART_Type *pUSARTx);
void HAL_USART_ClearBKFlag(LPUART_Type *pUSARTx);


void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
#endif /* INC_HAL_USART_H_ */
