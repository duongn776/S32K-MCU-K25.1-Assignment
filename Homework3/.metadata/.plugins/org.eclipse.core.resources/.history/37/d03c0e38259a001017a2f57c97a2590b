/*
 * HWAccess_GPIO.h
 *
 *  Created on: Sep 21, 2025
 *      Author: nhduo
 */

#ifndef INC_HWACCESS_GPIO_H_
#define INC_HWACCESS_GPIO_H_

#include "S32K144.h"

/**
 * @brief Set the specified GPIO pin high.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to set (0-31).
 */
static inline void GPIO_SetPin(GPIO_Type *pGPIOx, uint8_t pin)
{
	pGPIOx->PSOR = (1 << pin);
}

/**
 * @brief Set the specified GPIO pin low.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to clear (0-31).
 */
static inline void GPIO_ClearPin(GPIO_Type *pGPIOx, uint8_t pin)
{
	pGPIOx->PCOR = (1 << pin);
}

static inline void GPIO_TogglePin(GPIO_Type *pGPIOx, uint8_t pin)
{
	pGPIOx->PTOR = (1 << pin);
}

static inline void GPIO_WritePin(GPIO_Type *pGPIOx, uint8_t pin, uint8_t val)
{
	if (val)
	{
		pGPIOx->PSOR = (1 << pin);
	}else
	{
		pGPIOx->PCOR = (1 << pin);
	}
}

static inline uint8_t GPIO_ReadPin(GPIO_Type *pGPIOx, uint8_t pin)
{
	return (uint8_t)((pGPIOx->PDIR >> pin) & 0x1);
}

static inline void GPIO_SetPinDirection(GPIO_Type *pGPIOx, uint8_t pin, uint8_t dir)
{
	if (dir)
	{
		/* output direction */
		pGPIOx->PDDR |= (1 << pin);
	}else
	{
		/* input direction */
		pGPIOx->PDDR &= ~(1 << pin);
	}
}
#endif /* INC_HWACCESS_GPIO_H_ */
