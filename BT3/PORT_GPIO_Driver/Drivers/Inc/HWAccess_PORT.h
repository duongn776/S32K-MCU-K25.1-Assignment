/*
 * HWAccess_PORT.h
 *
 *  Created on: Sep 22, 2025
 *      Author: nhduong
 */

#ifndef INC_HWACCESS_PORT_H_
#define INC_HWACCESS_PORT_H_

#include "S32K144.h"

static inline void PORT_NoPull(PORT_Type *base, uint8_t pin)
{
	base->PCR[pin] &= ~PORT_PCR_PE_MASK;
}

static inline void PORT_PullUp(PORT_Type *base, uint8_t pin)
{
	base->PCR[pin] |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
}

static inline void PORT_PullDown(PORT_Type *base, uint8_t pin)
{
	base->PCR[pin] |= PORT_PCR_PE_MASK;
	base->PCR[pin] &= ~PORT_PCR_PS_MASK;
}

static inline void PORT_SetInterruptConfig(PORT_Type *base, uint8_t pin, uint8_t irqc)
{
	base->PCR[pin] = (base->PCR[pin] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(irqc);
}

static inline void PORT_ClearInterruptConfig(PORT_Type *base, uint8_t pin)
{
	base->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
}




#endif /* INC_HWACCESS_PORT_H_ */
