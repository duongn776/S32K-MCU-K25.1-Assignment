/*
 * Driver_PORT.c
 *
 *  Created on: Sep 21, 2025
 *      Author: nhduong
 */

#include "Driver_PORT.h"

/**
  * @brief  Enable or Disable clock for a PORT module
  * @param  base: PORTA, PORTB, PORTC, PORTD, PORTE
  * @param  clockState: 1 = enable, 0 = disable
  */
void PORT_PeriClockControl(PORT_Type *base, uint8_t clockState)
{
    uint32_t portIndex = 0;

    if(base == PORTA) portIndex = PCC_PORTA_INDEX;
    else if(base == PORTB) portIndex = PCC_PORTB_INDEX;
    else if(base == PORTC) portIndex = PCC_PORTC_INDEX;
    else if(base == PORTD) portIndex = PCC_PORTD_INDEX;
    else if(base == PORTE) portIndex = PCC_PORTE_INDEX;
    else return;

    if(clockState) {
        PCC->PCCn[portIndex] |= PCC_PCCn_CGC_MASK;  // enable
    } else {
        PCC->PCCn[portIndex] &= ~PCC_PCCn_CGC_MASK; // disable
    }
}

/**
  * @brief  Initialize the PORT pin with the specified parameters in the cfg
  * @param  cfg: pointer to a PORT_Config_t structure that contains
  *         the configuration information for the specified PORT pin
  * @note   This function configures the pin once. To reconfigure, call this function again.
  */
void PORT_Init(const PORT_Config_t *cfg)
{
    uint32_t reg = 0U;

    /* MUX setting */
    reg |= PORT_PCR_MUX(cfg->mux);

    /* Pull resistor */
    switch (cfg->pull) {
        case PORT_PULLUP:
            reg |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
            break;
        case PORT_PULLDOWN:
            reg |= PORT_PCR_PE_MASK;
            reg &= ~PORT_PCR_PS_MASK;
            break;
        case PORT_NOPULL:
        default:
            reg &= ~PORT_PCR_PE_MASK;
            break;
    }

    /* Interrupt config */
    if (cfg->interrupt != PORT_INT_DISABLED) {
        reg = (reg & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(cfg->interrupt);
    }

    /* Write config to PCR once */
    cfg->portBase->PCR[cfg->pin] = reg;
}




