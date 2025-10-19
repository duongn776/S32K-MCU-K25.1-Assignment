/*
 * HAL_USART.c
 *
 *  Created on: Oct 9, 2025
 *      Author: nhduong
 */

#include "HAL_USART.h"

static void HAL_USART_EndTxTransfer(USART_HandleTypeDef *husart);
static void HAL_USART_Transmit_TXE(USART_HandleTypeDef *husart);
static void HAL_USART_Receive_RXNE(USART_HandleTypeDef *husart);
/**
 * @brief Enable or disable the peripheral clock for the specified USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 * @param state state of the peripheral clock (ENABLE or DISABLE).
 */
void HAL_USART_PeriClockControl(LPUART_Type *pUSARTx, uint8_t state)
{
	if (state)
	{
		if (pUSARTx == LPUART0)
		{
			PCC->PCCn[PCC_LPUART0_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART1)
		{
			PCC->PCCn[PCC_LPUART1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART2)
		{
			PCC->PCCn[PCC_LPUART2_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
		}
	}
	else
	{
		if (pUSARTx == LPUART0)
		{
			PCC->PCCn[PCC_LPUART0_INDEX] &= PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART1)
		{
			PCC->PCCn[PCC_LPUART1_INDEX] &= PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART2)
		{
			PCC->PCCn[PCC_LPUART2_INDEX] &= PCC_PCCn_CGC_MASK;
		}
	}
}

/**
 * @brief Configure the GPIO pins for the specified USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
static void HAL_USART_ConfigPin(LPUART_Type *pUSARTx)
{
	if (pUSARTx == LPUART0)
	{
		/* Config PTB0 as LPUART1_RX */
		PORTB->PCR[0] = PORT_PCR_MUX(3);
		/* Config PTB1 as LPUART1_TX */
		PORTB->PCR[1] = PORT_PCR_MUX(3);
	}
	else if (pUSARTx == LPUART1)
	{

		/* Config PTC8 as LPUART0_RX */
		PORTC->PCR[8] = PORT_PCR_MUX(2);
		/* Config PTC9 as LPUART0_TX */
		PORTC->PCR[9] = PORT_PCR_MUX(2);
	}
	else if (pUSARTx == LPUART2)
	{
		/* Config PTD2 as LPUART2_RX */
		PORTD->PCR[2] = PORT_PCR_MUX(3);
		/* Config PTD3 as LPUART2_TX */
		PORTD->PCR[3] = PORT_PCR_MUX(3);
	}
}

/**
 * @brief Reset the USART peripheral.
 * 
 * @param pUSARTx pointer to the USART peripheral.
 */
static void HAL_USART_Reset(LPUART_Type *pUSARTx)
{
	/* Reset LPUART module */
    pUSARTx->GLOBAL = 0x0;
    pUSARTx->CTRL   = 0x0;
    pUSARTx->BAUD   = 0x0;
    pUSARTx->STAT   = 0xC01FC000;
    pUSARTx->MODIR  = 0x0;
}

/**
 * @brief Control the clock for the USART port.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 * @param state State of the clock (ENABLE or DISABLE).
 */
void HAL_USART_PortClockControl(LPUART_Type *pUSARTx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pUSARTx == LPUART0)
		{
			PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART1)
		{
			// PCC->PCCn[PCC_PORTc_INDEX] |= PCC_PCCn_CGC_MASK;
			PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART2)
		{
			PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
		}
		else
		{
			/* Do nothing */
		}
	}
	else if (state == DISABLE)
	{
		if (pUSARTx == LPUART0)
		{
			PCC->PCCn[PCC_PORTC_INDEX] &= ~PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART1)
		{	
			PCC->PCCn[PCC_PORTB_INDEX] &= ~PCC_PCCn_CGC_MASK;
		}
		else if (pUSARTx == LPUART2)
		{
			PCC->PCCn[PCC_PORTD_INDEX] &= ~PCC_PCCn_CGC_MASK;
		}
		else
		{
			/* Do nothing */
		}
	}
}
/**
 * @brief Initialize the USART peripheral.
 * 
 * @param husart Pointer to the USART handle.
 */
void HAL_USART_Init(USART_HandleTypeDef *husart)
{
	/* Enable clock for LPUART */
	HAL_USART_PeriClockControl(husart->pUSARTx, ENABLE);

	/* Enable clock for Port */
	HAL_USART_PortClockControl(husart->pUSARTx, ENABLE);

	/* Config Pin */
	HAL_USART_ConfigPin(husart->pUSARTx);

	/* Reset LPUART before initialization */
	HAL_USART_Reset(husart->pUSARTx);

	/* Clear any pending interrupts and enable NVIC */
	NVIC_ClearPendingIRQ(LPUART1_RxTx_IRQn);
	NVIC_EnableIRQ(LPUART1_RxTx_IRQn);
}

void HAL_USART_DeInit(USART_HandleTypeDef *husart)
{
	/* Reset LPUART peripheral */
	HAL_USART_Reset(husart->pUSARTx);

	/* Disable clock for LPUART */
	HAL_USART_PeriClockControl(husart->pUSARTx, DISABLE);

	/* Disable clock for Port */
	HAL_USART_PortClockControl(husart->pUSARTx, DISABLE);
}
/**
 * @brief Set the baud rate for the specified USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 * @param BaudRate Desired baud rate.
 */
void USART_SetBaudRate(LPUART_Type *pUSARTx, uint32_t BaudRate)
{
    uint32_t osr = 15; // 16x oversampling
    uint32_t sbr = (FREQ_USART_CLK / ((osr + 1) * BaudRate));
    pUSARTx->BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
}

/**
 * @brief Transmit data using interrupt mode
 * 
 * @param husart Handle to the USART peripheral.
 * @param pTxBuffer Pointer to the data buffer to be transmitted.
 * @param Len Length of the data to be transmitted.
 */
void HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxBuffer, uint32_t Len)
{
	const uint16_t *pTxData16bits;

	for (uint32_t i = 0; i < Len; i++)
	{
		/* Wait until transmit data register is empty */
		while (!(husart->pUSARTx->STAT & LPUART_STAT_TDRE_MASK));

		/* Check word length (9 bits or 8 bits) */
		if (husart->Init.WordLength == USART_WORDLENGTH_9BITS)
		{
			/* Transmit 9 bits data, So load DR with 2 bytes */
			pTxData16bits = (uint16_t *)pTxBuffer;
			husart->pUSARTx->DATA = (*pTxData16bits & (0x01FF));

			/* Check for parity bit control */
			if (husart->Init.ParityControl == USART_PARITY_NONE)
			{
				/* No parity, so 9 bits of user data */
				/* Increment buffer to point to next 2 bytes */
				pTxBuffer += 2;
			}
			else
			{
				/* With parity, so 8 bits of user data */
				/* Increment buffer to point to next byte */
				pTxBuffer++;
			}
		}
		else
		{
			/* Transmit 8 bits data, So load DR with 1 byte */
			husart->pUSARTx->DATA = (*pTxBuffer & (0x00FF));
			/* Increment buffer to point to next byte */
			pTxBuffer++;
		}
	}

	/* Wait until transmission is complete */
	while (!(husart->pUSARTx->STAT & LPUART_STAT_TC_MASK));
}

/**
 * @brief Receive data using interrupt mode.
 * 
 * @param husart Handle to the USART peripheral.
 * @param pRxBuffer Pointer to the data buffer to be received.
 * @param Len Length of the data to be received.
 */
void HAL_USART_Receive(USART_HandleTypeDef *husart,uint8_t *pRxBuffer, uint32_t Len)
{
	/* Loop until all data is received */
	for (uint32_t i = 0; i < Len; i++)
	{
		/* Wait until receive data register is full */
		while (!(husart->pUSARTx->STAT & LPUART_STAT_RDRF_MASK));

		/* Check word length (9 bits or 8 bits) */
		if (husart->Init.WordLength == USART_WORDLENGTH_9BITS)
		{
			/* Check for parity bit control */
			if (husart->Init.ParityControl == USART_PARITY_NONE)
			{
				/* No parity, so 9 bits of user data */
				/* Receive 9 bits data, So read DR with 2 bytes */
				*((uint16_t *)pRxBuffer) = (uint16_t)(husart->pUSARTx->DATA & (0x01FF));
				/* Increment buffer to point to next 2 bytes */
				pRxBuffer += 2;
			}
			else
			{
				/* With parity, so 8 bits of user data */
				/* Receive 8 bits data, So read DR with 1 byte */
				*pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x00FF));
				/* Increment buffer to point to next byte */
				pRxBuffer++;
			}
		}
		else
		{
			if (husart->Init.ParityControl == USART_PARITY_NONE)
			{
				/* No parity, so 8 bits of user data */
				/* Receive 8 bits data, So read DR with 1 byte */
				*pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x00FF));
			}
			else
			{
				/* With parity, so 7 bits of user data */
				/* Receive 7 bits data, So read DR with 1 byte */
				*pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x007F));
			}
			pRxBuffer++;
		}
	}
}


/**
 * @brief Set the USART to 7-bit mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_Set7BitMode(LPUART_Type *pUSARTx)
{
	/* Clear 9 bit mode */
	pUSARTx->CTRL &= ~LPUART_CTRL_M_MASK;
	/* Set 7 bit mode */
	pUSARTx->CTRL |= LPUART_CTRL_M7_MASK;
}

/**
 * @brief Set the USART to 8-bit mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_Set8BitMode(LPUART_Type *pUSARTx)
{
	/* Clear and set 8 bit mode */
	pUSARTx->CTRL &= ~LPUART_CTRL_M_MASK;
	/* clear 7 bit mode */
	pUSARTx->CTRL &= ~LPUART_CTRL_M7_MASK;
	
}

/**
 * @brief Set the USART to 9-bit mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_Set9BitMode(LPUART_Type *pUSARTx)
{
	/* Set 9 bit mode */
	pUSARTx->CTRL |= LPUART_CTRL_M_MASK;
	/* Clear 7 bit mode */
	pUSARTx->CTRL &= ~LPUART_CTRL_M7_MASK;
}

/**
 * @brief Set the USART to None parity mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_SetParityNone(LPUART_Type *pUSARTx)
{
	/* Clear parity enable bit */
	pUSARTx->CTRL &= ~LPUART_CTRL_PE_MASK;
}

/**
 * @brief Set the USART to Even parity mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_SetParityEven(LPUART_Type *pUSARTx)
{
	/* Set parity enable bit */
	pUSARTx->CTRL |= LPUART_CTRL_PE_MASK;
	/* Set even parity */
	pUSARTx->CTRL &= ~LPUART_CTRL_PT_MASK;
}

/**
 * @brief Set the USART to Odd parity mode.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_SetParityOdd(LPUART_Type *pUSARTx)
{
	/* Set parity enable bit */
	pUSARTx->CTRL |= LPUART_CTRL_PE_MASK;
	/* Set odd parity */
	pUSARTx->CTRL |= LPUART_CTRL_PT_MASK;
}

/**
 * @brief Set the USART to 1. Stop bit.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_SetStopBits1(LPUART_Type *pUSARTx)
{
	pUSARTx->BAUD &= ~LPUART_BAUD_SBNS_MASK;
}

/**
 * @brief Set the USART to 2 Stop bits.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_SetStopBits2(LPUART_Type *pUSARTx)
{
	pUSARTx->BAUD |= LPUART_BAUD_SBNS_MASK;
}

/**
 * @brief Set the USART to No flow control.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_FlowControlNone(LPUART_Type *pUSARTx)
{
	pUSARTx->MODIR &= ~(LPUART_MODIR_TXCTSE_MASK | LPUART_MODIR_RXRTSE_MASK);
}

/**
 * @brief Set the USART to RTS flow control.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_FlowControlRTS(LPUART_Type *pUSARTx)
{
	pUSARTx->MODIR |= LPUART_MODIR_RXRTSE_MASK;
	pUSARTx->MODIR &= ~LPUART_MODIR_TXCTSE_MASK;
}

/**
 * @brief Set the USART to CTS flow control.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_FlowControlCTS(LPUART_Type *pUSARTx)
{
	pUSARTx->MODIR |= LPUART_MODIR_TXCTSE_MASK;
	pUSARTx->MODIR &= ~LPUART_MODIR_RXRTSE_MASK;
}

/**
 * @brief Set the USART to RTS and CTS flow control.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_FlowControlRTS_CTS(LPUART_Type *pUSARTx)
{
	pUSARTx->MODIR |= (LPUART_MODIR_TXCTSE_MASK | LPUART_MODIR_RXRTSE_MASK);
}

/**
 * @brief Enable the USART transmitter.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_EnableTx(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL |= LPUART_CTRL_TE_MASK;
}

/**
 * @brief Disable the USART transmitter.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_DisableTx(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL &= ~LPUART_CTRL_TE_MASK;
}

/**
 * @brief Enable the USART receiver.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_EnableRx(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL |= LPUART_CTRL_RE_MASK;
}

/**
 * @brief Disable the USART receiver.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_DisableRx(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL &= ~LPUART_CTRL_RE_MASK;
}

/**
 * @brief Abort ongoing USART reception.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_AbortTx(USART_HandleTypeDef *husart)
{
	/* Disable Transmit */
	husart->pUSARTx->CTRL &= ~LPUART_CTRL_TE_MASK;

	(void)husart->pUSARTx->DATA;  // Clear TDRE flag
}

/**
 * @brief Abort ongoing USART transmission.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_AbortRx(USART_HandleTypeDef *husart)
{
	/* Disable Receive */
	husart->pUSARTx->CTRL &= ~LPUART_CTRL_RE_MASK;

	(void)husart->pUSARTx->DATA;  // Clear RDRF flag
}

/**
 * @brief Send a break condition on the USART.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_SendBreak(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL |= LPUART_CTRL_SBK_MASK;
}

/**
 * @brief Stop sending a break condition on the USART.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_StopBreak(USART_HandleTypeDef *husart)
{
	husart->pUSARTx->CTRL &= ~LPUART_CTRL_SBK_MASK;
}

/**
 * @brief Clear the Overrun flag of the USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_ClearORFlag(LPUART_Type *pUSARTx)
{
	pUSARTx->STAT |= LPUART_STAT_OR_MASK;
}

/**
 * @brief Clear the Framing Error flag of the USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_ClearFEFlag(LPUART_Type *pUSARTx)
{
	pUSARTx->STAT |= LPUART_STAT_FE_MASK;
}

/**
 * @brief Clear the Parity Error flag of the USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_ClearPFFlag(LPUART_Type *pUSARTx)
{
	pUSARTx->STAT |= LPUART_STAT_PF_MASK;
}

/**
 * @brief Clear the Break Detect flag of the USART.
 * 
 * @param pUSARTx Pointer to the USART peripheral.
 */
void HAL_USART_ClearBKFlag(LPUART_Type *pUSARTx)
{
	pUSARTx->STAT |= LPUART_STAT_LBKDIF_MASK;
}

/**
 * @brief Transmit data using interrupt mode.
 *
 * @param husart Handle to the USART peripheral.
 * @param pTxBuffer Pointer to the data buffer to be transmitted.
 * @param Len Length of the data to be transmitted.
 * @return uint8_t State of the transmission (READY or BUSY).
 *         This function returns the previous state of the transmission.
 */
uint8_t HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = husart->TxState;

	/* Check if USART is busy transmitting */
	if (state != USART_STATE_BUSY_TX)
	{
		husart->pTxBuffer = pTxBuffer;
		husart->TxLen = Len;
		husart->TxState = USART_STATE_BUSY_TX;

		/* Enable Transmit Data Register Empty Interrupt */
		husart->pUSARTx->CTRL |= LPUART_CTRL_TIE_MASK;

		/* Enable Transmission Complete Interrupt */
		husart->pUSARTx->CTRL |= LPUART_CTRL_TCIE_MASK;
	}
	return state;
}

/**
 * @brief Receive data using interrupt mode.
 *
 * @param husart Handle to the USART peripheral.
 * @param pRxBuffer Pointer to the data buffer to be received.
 * @param Len Length of the data to be received.
 * @return uint8_t State of the reception (READY or BUSY).
 *         This function returns the previous state of the reception.
 */
uint8_t HAL_USART_Receive_IT(USART_HandleTypeDef *husart,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = husart->RxState;
	if (state != USART_STATE_BUSY_RX)
	{
		husart->pRxBuffer = pRxBuffer;
		husart->RxLen = Len;
		husart->RxState = USART_STATE_BUSY_RX;

		(void)husart->pUSARTx->DATA;  // Clear RDRF flag

		/* Enable Receive Data Register Full Interrupt */
		husart->pUSARTx->CTRL |= LPUART_CTRL_RIE_MASK;
	}
	return state;
}

/**
 * @brief  Handle USART event and error interrupt request.
 * 
 * @param husart Handle to the USART peripheral.
 */
void HAL_USART_IRQHandler(USART_HandleTypeDef *husart)
{
	uint32_t temp1, temp2;

	/* Check the state of TC in SR */
	temp1 = husart->pUSARTx->STAT & LPUART_STAT_TC_MASK;
	temp2 = husart->pUSARTx->CTRL & LPUART_CTRL_TCIE_MASK;

	if (temp1 && temp2)
	{
		/* This interrupt because of TC */
		if (husart->TxState == USART_STATE_BUSY_TX)
		{
			HAL_USART_EndTxTransfer(husart);
		}
	}

	/*************************Check for TXE flag ********************************************/
	temp1 = husart->pUSARTx->STAT & LPUART_STAT_TDRE_MASK;
	temp2 = husart->pUSARTx->CTRL & LPUART_CTRL_TIE_MASK;

	if (temp1 && temp2)
	{
		/* This interrupt because of TXE */
		if (husart->TxState == USART_STATE_BUSY_TX)
		{
			HAL_USART_Transmit_TXE(husart);
		}
	}

	/*************************Check for RXNE flag ********************************************/
	temp1 = husart->pUSARTx->STAT & LPUART_STAT_RDRF_MASK;
	temp2 = husart->pUSARTx->CTRL & LPUART_CTRL_RIE_MASK;

	if (temp1 && temp2)
	{
		/* This interrupt because of RXNE */
		if (husart->RxState == USART_STATE_BUSY_RX)
		{
			HAL_USART_Receive_RXNE(husart);
		}
	}
}

/**
  * @brief  End ongoing Tx transfer on USART peripheral (following error detection or Transmit completion).
  * @param  husart USART handle.
  * @retval None
  */
static void HAL_USART_EndTxTransfer(USART_HandleTypeDef *husart)
{
	/* Check Len */
	if (!(husart->TxLen))
	{
		/* Disable the TCIE control bit */
		husart->pUSARTx->CTRL &= ~LPUART_CTRL_TCIE_MASK;

		husart->TxState = USART_STATE_READY;
		husart->TxLen = 0;
		husart->pTxBuffer = NULL;

		/* Call the application callback to notify that transmission is complete */
		husart->CallbackEvent(USART_EVENT_TX_COMPLETE);
	}
}

/**
  * @brief  Handles the TXE (Transmit Data Register Empty) interrupt for the USART peripheral.
  * @param  husart Pointer to the USART_HandleTypeDef structure
  * 			   that contains the configuration information for the specified USART.
  * @retval None
  */
static void HAL_USART_Transmit_TXE(USART_HandleTypeDef *husart)
{
	uint16_t *pTxData16bits;

	/* Check state */
	if (husart->TxState == USART_STATE_BUSY_TX)
	{
		/* Check if there is data to send */
		if (husart->TxLen > 0)
		{
			/* Check word length (9 bits or 8 bits) */
			if (husart->Init.WordLength == USART_WORDLENGTH_9BITS)
			{
				/* Transmit 9 bits data, So load DR with 2 bytes */
				pTxData16bits = (uint16_t *)husart->pTxBuffer;
				husart->pUSARTx->DATA = (*pTxData16bits & (0x01FF));

				/* Check for parity bit control */
				if (husart->Init.ParityControl == USART_PARITY_NONE)
				{
					/* No parity, so 9 bits of user data */
					/* Increment buffer to point to next 2 bytes */
					husart->pTxBuffer += 2;
					husart->TxLen -= 2;
				}
				else
				{
					/* With parity, so 8 bits of user data */
					/* Increment buffer to point to next byte */
					husart->pTxBuffer++;
					husart->TxLen--;
				}
			}
			else
			{
				/* Transmit 8 bits data, So load DR with 1 byte */
				husart->pUSARTx->DATA = (*husart->pTxBuffer & (0x00FF));
				/* Increment buffer to point to next byte */
				husart->pTxBuffer++;
				husart->TxLen--;
			}
		}

		if (husart->TxLen == 0)
		{
			/* Disable the TCIE status bit */
			husart->pUSARTx->CTRL &= ~LPUART_CTRL_TCIE_MASK;
		}
	}
	
}

/**
  * @brief  Handles the RXNE (Receive Data Register Not Empty) interrupt for the USART peripheral.
  * @param  husart Pointer to the USART_HandleTypeDef structure
  * 			   that contains the configuration information for the specified USART.
  * @retval None
  */
static void HAL_USART_Receive_RXNE(USART_HandleTypeDef *husart)
{
	/* Check state */
	if (husart->RxState == USART_STATE_BUSY_RX)
	{
		/* Check if there is data to receive */
		if (husart->RxLen > 0)
		{
			/* Check word length (9 bits or 8 bits) */
			if (husart->Init.WordLength == USART_WORDLENGTH_9BITS)
			{
				/* Check for parity bit control */
				if (husart->Init.ParityControl == USART_PARITY_NONE)
				{
					/* No parity, so 9 bits of user data */
					/* Receive 9 bits data, So read DR with 2 bytes */
					*((uint16_t *)husart->pRxBuffer) = (uint16_t)(husart->pUSARTx->DATA & (0x01FF));
					/* Increment buffer to point to next 2 bytes */
					husart->pRxBuffer += 2;
					husart->RxLen -= 2;
				}
				else
				{
					/* With parity, so 8 bits of user data */
					/* Receive 8 bits data, So read DR with 1 byte */
					*husart->pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x00FF));
					/* Increment buffer to point to next byte */
					husart->pRxBuffer++;
					husart->RxLen--;
				}
			}
			else
			{
				if (husart->Init.ParityControl == USART_PARITY_NONE)
				{
					/* No parity, so 8 bits of user data */
					/* Receive 8 bits data, So read DR with 1 byte */
					*husart->pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x00FF));
				}
				else
				{
					/* With parity, so 7 bits of user data */
					/* Receive 7 bits data, So read DR with 1 byte */
					*husart->pRxBuffer = (uint8_t)(husart->pUSARTx->DATA & (0x007F));
				}
				husart->pRxBuffer++;
				husart->RxLen--;
			}
		}
		if (husart->RxLen == 0)
		{
			/* Disable the RDRF status bit */
			husart->pUSARTx->CTRL &= ~LPUART_CTRL_RIE_MASK;

			husart->RxState = USART_STATE_READY;
			husart->RxLen = 0;
			husart->pRxBuffer = NULL;

			/* Call the application callback to notify that reception is complete */
			husart->CallbackEvent(USART_EVENT_RECEIVE_COMPLETE);
		}
	}
}
