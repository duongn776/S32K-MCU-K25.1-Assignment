/*
 * Driver_USART.c
 *
 *  Created on: Oct 6, 2025
 *      Author: nhduong
 */
#include "Driver_USART.h"

static USART_HandleTypeDef huart1 = {
    .pUSARTx = LPUART1,
    .Init = {
        .Mode = USART_MODE_TX_RX,
        .BaudRate = USART_BAUDRATE_115200,
        .StopBits = USART_STOPBITS_1,
        .ParityControl = USART_PARITY_NONE,
        .HWFlowControl = USART_HW_NONE
    },
	.pTxBuffer = NULL,
	.pRxBuffer = NULL,
	.TxLen = 0,
	.RxLen = 0,
	.TxState = USART_STATE_READY,
	.RxState = USART_STATE_READY,
	.CallbackEvent = NULL
};
#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

//
//   Functions
//

static ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
{
  return DriverVersion;
}

static ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
{
  return DriverCapabilities;
}

/**
 * @brief Initialize the USART interface
 * 
 * @param cb_event pointer to ARM_USART_SignalEvent callback function
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported
 */
static int32_t ARM_USART_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    if (cb_event != NULL)
    {
        huart1.CallbackEvent = cb_event;
    }
    
    /* Initialize USART peripheral */
    HAL_USART_Init(&huart1);

    return ARM_DRIVER_OK;
}

/**
 * @brief Uninitialize the USART interface
 * 
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported
 */
static int32_t ARM_USART_Uninitialize(void)
{
    /* De-initialize USART peripheral */
    HAL_USART_DeInit(&huart1);

    return ARM_DRIVER_OK;
}


/**
 * @brief Control the power state of the USART interface
 * 
 * @param state Power state to set
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported

    @note: This function currently does not implmement any power control functionality.
          It simply returns ARM_DRIVER_OK for all power states.
 */
static int32_t ARM_USART_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        /* Nothing to do */
        break;

    case ARM_POWER_LOW:
        /* Nothing to do */
        break;

    case ARM_POWER_FULL:
        /* Nothing to do */
        break;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief Start sending data to USART transmitter
 * 
 * @param data Pointer to the data buffer to be sent.
 * @param num Number of bytes to send.
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported
 */
static int32_t ARM_USART_Send(const void *data, uint32_t num)
{
    const uint8_t *pData = (const uint8_t *)data;

    /* Check for valid parameters */
    if ((pData == NULL) || (num == 0U))
    {
        return ARM_DRIVER_ERROR;
    }

    /* Blocking mode */
    HAL_USART_Transmit(&huart1, (uint8_t*)pData, num);
    return ARM_DRIVER_OK;
}

/**
 * @brief Start receiving data from USART receiver
 * 
 * @param data Pointer to the data buffer to store received data.
 * @param num Number of bytes to receive.
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported
 */
static int32_t ARM_USART_Receive(void *data, uint32_t num)
{
    uint8_t *pData = (uint8_t *)data;

    /* Check for valid parameters */
    if ((pData == NULL) || (num == 0U))
    {
        return ARM_DRIVER_ERROR;
    }

    if (huart1.CallbackEvent != NULL)
    {
        /* Interrupt mode */
        HAL_USART_Receive_IT(&huart1, pData, num);
    }
    else
    {
        /* Blocking mode */
        HAL_USART_Receive(&huart1, pData, num);
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief Start sending/receiving data to/from USART transmitter/receiver
 * 
 * @param data_out Pointer to the data buffer to be sent.
 * @param data_in Pointer to the data buffer to store received data.
 * @param num Number of bytes to transfer.
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
  - ARM_DRIVER_OK: Operation successful
  - ARM_DRIVER_ERROR: Unspecified error
  - ARM_DRIVER_ERROR_BUSY: Driver is busy
  - ARM_DRIVER_ERROR_UNSUPPORTED: Operation not supported

    @note: This function currently does not implement any transfer functionality.
          It simply returns ARM_DRIVER_OK.
 */
static int32_t ARM_USART_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Get the number of data items transmitted
 * 
 * @return uint32_t Number of data items transmitted
 *       @note: This function currently does not implement any functionality to track transmitted data count.
 */
static uint32_t ARM_USART_GetTxCount(void)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Get the number of data items received
 * 
 * @return uint32_t Number of data items received
 *       @note: This function currently does not implement any functionality to track received data count.
 */
static uint32_t ARM_USART_GetRxCount(void)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Control USART Interface
 * 
 * @param control Operation control code
 *  @Note: Only support Asynchronous mode
  - \ref ARM_USART_MODE_ASYNCHRONOUS
 * @param arg Argument for the control operation
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure
 */
static int32_t ARM_USART_Control(uint32_t control, uint32_t arg)
{
    uint32_t operation_mode = control & ARM_USART_MODE_ASYNCHRONOUS;
    uint32_t data_bits = control & ARM_USART_DATA_BITS_Msk;
    uint32_t parity_bit = control & ARM_USART_PARITY_Msk;
    uint32_t stop_bits = control & ARM_USART_STOP_BITS_Msk;
    uint32_t flow_control = control & ARM_USART_FLOW_CONTROL_Msk;
    uint32_t clock_parity = control & ARM_USART_CPOL_Msk;
    uint32_t clock_phase = control & ARM_USART_CPHA_Msk;
    uint32_t miscellaneous = control & ARM_USART_CONTROL_Msk;
    if ((arg != 0) && (arg != 1))
    {
        /* Configure Operation mode */
        switch (operation_mode)
        {
            case ARM_USART_MODE_ASYNCHRONOUS:
                // Configure for Asynchronous mode
                huart1.Init.BaudRate = arg;
                USART_SetBaudRate(huart1.pUSARTx, huart1.Init.BaudRate);
                break;

            case ARM_USART_MODE_SYNCHRONOUS_MASTER:
            case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
            case ARM_USART_MODE_SINGLE_WIRE:
            case ARM_USART_MODE_IRDA:
            case ARM_USART_MODE_SMART_CARD:
            default:
                /* unsupported mode */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Configure Data bits */
        switch (data_bits)
        {
            case ARM_USART_DATA_BITS_7:
                /* Set 7-bit data mode */
                HAL_USART_Set7BitMode(huart1.pUSARTx);
                break;

            case ARM_USART_DATA_BITS_8:
                /* Set 8-bit data mode */
                HAL_USART_Set8BitMode(huart1.pUSARTx);
                break;

            case ARM_USART_DATA_BITS_9:
                /* Set 9-bit data mode */
                HAL_USART_Set9BitMode(huart1.pUSARTx);
                break;

            case ARM_USART_DATA_BITS_5:
            case ARM_USART_DATA_BITS_6:
            default:
                /* unsupported data bits */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Configure Parity bit */
        switch (parity_bit)
        {
            case ARM_USART_PARITY_NONE:
                /* Set no parity bit */
                HAL_USART_SetParityNone(huart1.pUSARTx);
                break;

            case ARM_USART_PARITY_EVEN:
                /* Set even parity bit */
                HAL_USART_SetParityEven(huart1.pUSARTx);
                break;

            case ARM_USART_PARITY_ODD:
                /* Set odd parity bit */
                HAL_USART_SetParityOdd(huart1.pUSARTx);
                break;
            default:
                /* unsupported parity bit */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Configure Stop bits */
        switch (stop_bits)
        {
            case ARM_USART_STOP_BITS_1:
                /* Set 1 stop bit */
                HAL_USART_SetStopBits1(huart1.pUSARTx);
                break;

            case ARM_USART_STOP_BITS_2:
                /* Set 2 stop bits */
                HAL_USART_SetStopBits2(huart1.pUSARTx);
                break;

            case ARM_USART_STOP_BITS_1_5:
            case ARM_USART_STOP_BITS_0_5:
            default:
                /* unsupported stop bits */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Configure Flow control */
        switch (flow_control)
        {
            case ARM_USART_FLOW_CONTROL_NONE:
                /* No flow control */
                HAL_USART_FlowControlNone(huart1.pUSARTx);
                break;

            case ARM_USART_FLOW_CONTROL_RTS:
                /* RTS flow control */
                HAL_USART_FlowControlRTS(huart1.pUSARTx);
                break;

            case ARM_USART_FLOW_CONTROL_CTS:
                /* CTS flow control */
                HAL_USART_FlowControlCTS(huart1.pUSARTx);
                break;

            case ARM_USART_FLOW_CONTROL_RTS_CTS:
                /* RTS and CTS flow control */
                HAL_USART_FlowControlRTS_CTS(huart1.pUSARTx);
                break;

            default:
                /* unsupported flow control */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Configure clock polarity and clock phase */
        /* @Note: These parameters are only applicable for synchronous mode, so not implemented */
        if ((clock_parity != 0U) || (clock_phase != 0U))
        {
            /* unsupported clock polarity or clock phase */
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    /* Configure miscellaneous */
    switch (miscellaneous)
    {
        case ARM_USART_CONTROL_TX:
            if (arg)
            {
                /* Enable transmitter */
                HAL_USART_EnableTx(&huart1);
            }
            else
            {
                /* Disable transmitter */
                HAL_USART_DisableTx(&huart1);
            }
            break;

        case ARM_USART_CONTROL_RX:
            if (arg)
            {
                /* Enable receiver */
                HAL_USART_EnableRx(&huart1);
            }
            else
            {
                /* Disable receiver */
                HAL_USART_DisableRx(&huart1);
            }
            break;
        
        case ARM_USART_ABORT_SEND:
            /* Abort ongoing USART transmission */
            HAL_USART_AbortTx(&huart1);
            break;
        
        case ARM_USART_ABORT_RECEIVE:
            /* Abort ongoing USART reception */
            HAL_USART_AbortRx(&huart1);
            break;
        
        case ARM_USART_ABORT_TRANSFER:
            /* Abort ongoing USART transmission and reception */
            HAL_USART_AbortTx(&huart1);
            HAL_USART_AbortRx(&huart1);
            break;
        
        case ARM_USART_CONTROL_BREAK:
            if (arg)
            {
                /* Send break */
                HAL_USART_SendBreak(&huart1);
            }
            else
            {
                /* No break */
                HAL_USART_StopBreak(&huart1);
            }
            break;
        
            /* Unsupported features on S32K144 */
            case ARM_USART_CONTROL_SMART_CARD_NACK:
            case ARM_USART_SET_DEFAULT_TX_VALUE:
            case ARM_USART_SET_IRDA_PULSE:
            case ARM_USART_SET_SMART_CARD_CLOCK:
            case ARM_USART_SET_SMART_CARD_GUARD_TIME:
                return ARM_DRIVER_ERROR_UNSUPPORTED;

            default:
                /* unsupported miscellaneous operation */
                return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief Get USART status
 * 
 * @return ARM_USART_STATUS 
 */
static ARM_USART_STATUS ARM_USART_GetStatus(void)
{
    ARM_USART_STATUS status = {0};
    uint32_t stat_reg = huart1.pUSARTx->STAT;

    /* TX & RX busy flags (software-level) */
    status.tx_busy = (huart1.TxState == USART_STATE_BUSY_TX) ? 1U : 0U;
    status.rx_busy = (huart1.RxState == USART_STATE_BUSY_RX) ? 1U : 0U;

    /* Hardware-level error flags */
    if (stat_reg & LPUART_STAT_OR_MASK)
    {
        status.rx_overflow = 1U;
        /* Clear Overrun flag */
        HAL_USART_ClearORFlag(huart1.pUSARTx);
    }

    if (stat_reg & LPUART_STAT_FE_MASK)
    {
        status.rx_framing_error = 1U;
        /* Clear Framing Error flag */
        HAL_USART_ClearFEFlag(huart1.pUSARTx);
    }

    if (stat_reg & LPUART_STAT_PF_MASK)
    {
        status.rx_parity_error = 1U;
        /* Clear Parity Error flag */
        HAL_USART_ClearPFFlag(huart1.pUSARTx);
    }

    if (stat_reg & LPUART_STAT_LBKDIF_MASK)
    {
        status.rx_break = 1U;
        /* Clear Break Detect flag */
        HAL_USART_ClearBKFlag(huart1.pUSARTx);
    }

    return status;
}

/**
 * @brief Set the modem control signals for the USART.
 * 
 * @param control 
 * @return int32_t 
 * @Note This function currently does not implement any modem control functionality.
 */
static int32_t ARM_USART_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Get the modem status for the USART.
 * 
 * @return ARM_USART_MODEM_STATUS 
 * @Note This function currently does not implement any modem status functionality.
 */
static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus(void)
{
    ARM_USART_MODEM_STATUS status = {0};
    return status;
}

/**
 * @brief Signal USART events
 * 
 * @param event event code
  - \ref ARM_USART_EVENT_SEND_COMPLETE
  - \ref ARM_USART_EVENT_RECEIVE_COMPLETE
  - \ref ARM_USART_EVENT_TRANSFER_COMPLETE
  - \ref ARM_USART_EVENT_TX_COMPLETE
  - \ref ARM_USART_EVENT_RX_TIMEOUT
  - \ref ARM_USART_EVENT_RX_OVERFLOW
  - \ref ARM_USART_EVENT_TX_UNDERFLOW
  - \ref ARM_USART_EVENT_RX_BREAK
  - \ref ARM_USART_EVENT_RX_FRAMING_ERROR
  - \ref ARM_USART_EVENT_RX_PARITY_ERROR
  - \ref ARM_USART_EVENT_CTS
  - \ref ARM_USART_EVENT_DSR
  - \ref ARM_USART_EVENT_DCD
  - \ref ARM_USART_EVENT_RI

    @note This function is called by the HAL USART driver to signal events to the upper layer.
          It simply forwards the event to the callback function registered during initialization.
 */
static void ARM_USART_SignalEvent(uint32_t event)
{
    // function body
	(void)huart1.CallbackEvent(event);
}


void LPUART1_RxTx_IRQHandler(void)
{
    HAL_USART_IRQHandler(&huart1);
}

// End USART Interface

extern \
ARM_DRIVER_USART Driver_USART1;
ARM_DRIVER_USART Driver_USART1 = {
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    ARM_USART_Initialize,
    ARM_USART_Uninitialize,
    ARM_USART_PowerControl,
    ARM_USART_Send,
    ARM_USART_Receive,
    ARM_USART_Transfer,
    ARM_USART_GetTxCount,
    ARM_USART_GetRxCount,
    ARM_USART_Control,
    ARM_USART_GetStatus,
    ARM_USART_SetModemControl,
    ARM_USART_GetModemStatus
};
