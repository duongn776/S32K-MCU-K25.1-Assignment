/**
 ******************************************************************************
 * @file    HAL_GPIO.c
 * @author  Nguyen Hoang Duong
 * @date    06-Oct-2025
 * @brief   GPIO & PORT HAL driver for NXP S32K144
 *
 * Provides hardware-level access for GPIO pin control, pull configuration,
 * and interrupt event handling.
 ******************************************************************************
 */
#include "HAL_GPIO.h"

/* Base address arrays for PORTx and GPIOx */
PORT_Type * const PORT_BASE[] = IP_PORT_BASE_PTRS;
GPIO_Type * const GPIO_BASE[] = IP_GPIO_BASE_PTRS;

/* ===========================================================
 *                      PRIVATE VARIABLES
 * =========================================================== */
static HAL_GPIO_SignalEvent_t GPIO_Callbacks[GPIO_MAX_PINS] = {0};

/**
 * @brief  Setup a GPIO pin with default configuration.
 *         - Configures pin as input, no pull resistor, no interrupt.
 * @param  pin       GPIO pin number
 * @param  cb_event  Optional callback for pin interrupt events
 * @retval ARM_DRIVER_OK or error code
 */
void GPIO_Init(uint32_t pin, HAL_GPIO_SignalEvent_t cb_event)
{
    if (pin >= GPIO_MAX_PINS) {
        return; // Invalid pin number
    }

    uint8_t port_index = GET_PORT(pin);
    uint8_t pin_num = GET_PIN(pin);

     /* save callback */
    if (cb_event != NULL)
    {
        GPIO_Callbacks[pin] = cb_event;
    }


    /* Enable clock for the port */
    PORT_PeriClockControl(PORT_BASE[port_index], ENABLE);
     /* Config default of PCR */
     PORT_Config_t cfg = {
                .portBase      = PORT_BASE[port_index],
                .pin       = pin_num,
                .mux       = PORT_MUX_GPIO,
                .pull      = PORT_NOPULL,
                .interrupt = PORT_INT_DISABLED
    };
    PORT_Init(&cfg);

    /* Default: input direction */
    GPIO_SetPinDirection(GPIO_BASE[port_index], pin_num, PORT_INPUT);
}

/**
 * @brief Set the specified GPIO pin high.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to set (0-31).
 */
void GPIO_SetPin(GPIO_Type *pGPIOx, uint8_t pin)
{
    pGPIOx->PSOR = (1 << pin);
}

/**
 * @brief Set the specified GPIO pin low.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to clear (0-31).
 */
void GPIO_ClearPin(GPIO_Type *pGPIOx, uint8_t pin)
{
    pGPIOx->PCOR = (1 << pin);
}

/**
 * @brief Toggle the specified GPIO pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to toggle (0-31).
 */
void GPIO_TogglePin(GPIO_Type *pGPIOx, uint8_t pin)
{
    pGPIOx->PTOR = (1 << pin);
}

/**
 * @brief Write a value to the specified GPIO pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to write (0-31).
 * @param val Value to write (0 or 1).
 */
void GPIO_WritePin(GPIO_Type *pGPIOx, uint8_t pin, uint8_t val)
{
    if (val)
    {
        pGPIOx->PSOR = (1 << pin);
    }else
    {
        pGPIOx->PCOR = (1 << pin);
    }
}

/**
 * @brief Read the state of the specified GPIO pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to read (0-31).
 * @return uint8_t The state of the pin (0 or 1).
 */
uint8_t GPIO_ReadPin(GPIO_Type *pGPIOx, uint8_t pin)
{
    return (uint8_t)((pGPIOx->PDIR >> pin) & 0x1);
}

/**
 * @brief Configure the direction of the specified GPIO pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral base address.
 * @param pin The pin number to configure (0-31).
 * @param dir Direction to set (1 for output, 0 for input).
 */
void GPIO_SetPinDirection(GPIO_Type *pGPIOx, uint8_t pin, uint8_t dir)
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

/**
 * @brief Configure the specified pin to use no pull-up or pull-down resistor.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 */
void PORT_NoPull(PORT_Type *base, uint8_t pin)
{
    base->PCR[pin] &= ~PORT_PCR_PE_MASK;
}

/**
 * @brief Configure the specified pin to use pull-up resistor.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 */
void PORT_PullUp(PORT_Type *base, uint8_t pin)
{
    base->PCR[pin] |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
}

/**
 * @brief Configure the specified pin to use pull-down resistor.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 */
void PORT_PullDown(PORT_Type *base, uint8_t pin)
{
    base->PCR[pin] |= PORT_PCR_PE_MASK;
    base->PCR[pin] &= ~PORT_PCR_PS_MASK;
}

/**
 * @brief Configure the specified pin to use interrupt.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 * @param irqc The interrupt configuration (0-15).
 */
void PORT_SetInterruptConfig(PORT_Type *base, uint8_t pin, uint8_t irqc)
{
    base->PCR[pin] = (base->PCR[pin] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(irqc);
}

/**
 * @brief Clear the interrupt configuration for the specified pin.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 */
void PORT_ClearInterruptConfig(PORT_Type *base, uint8_t pin)
{
    base->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
}

/**
 * @brief Get the interrupt flags for the specified port.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @return uint32_t The interrupt flags.
 */
uint32_t PORT_GetInterruptFlags(PORT_Type *base)
{
    return base->ISFR;
}

/**
 * @brief Clear the interrupt flag for the specified pin.
 *
 * @param base Pointer to the PORT peripheral base address.
 * @param pin The pin number to configure (0-31).
 */
void PORT_ClearInterruptFlag(PORT_Type *base, uint8_t pin)
{
    base->ISFR = (1U << pin);  // Write 1 to clear
}

/**
  * @brief  Enable or Disable clock for a PORT module
  * @param  base: PORTA, PORTB, PORTC, PORTD, PORTE
  * @param  clockState: 1 = enable, 0 = disable
  */
void PORT_PeriClockControl(PORT_Type *base, uint8_t clockState)
{
    uint32_t portIndex = 0;

    if(base == PORTA)
    {
        portIndex = PCC_PORTA_INDEX;
    }
    else if(base == PORTB)
    {
        portIndex = PCC_PORTB_INDEX;
    }
    else if(base == PORTC)
    {
        portIndex = PCC_PORTC_INDEX;
    }
    else if(base == PORTD)
    {
        portIndex = PCC_PORTD_INDEX;
    }
    else if(base == PORTE)
    {
        portIndex = PCC_PORTE_INDEX;
    }
    else return;

    if(clockState)
    {
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
    switch (cfg->pull) 
    {
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
    if (cfg->interrupt != PORT_INT_DISABLED) 
    {
        reg = (reg & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(cfg->interrupt);
    }

    /* Write config to PCR once */
    cfg->portBase->PCR[cfg->pin] = reg;
}


/**
 * @brief  Common PORT interrupt handler for all ports (A-E).
 *         Reads ISFR, clears flags, and calls registered callbacks.
 *
 * @param  port_index: 0 = PORTA, 1 = PORTB, 2 = PORTC, 3 = PORTD, 4 = PORTE
 */
void PORT_IRQHandler(uint8_t port_index)
{
    PORT_Type *PORTx = PORT_BASE[port_index];
    uint32_t flags = PORTx->ISFR;  // read interrupt status flags
    PORTx->ISFR = flags;           // clear flags by writing 1

    for (uint8_t pin = 0; pin < 32; pin++)
    {
        if (flags & (1UL << pin))
        {
            uint32_t pin_index = port_index * 32U + pin;
            HAL_GPIO_SignalEvent_t cb = GPIO_Callbacks[pin_index];

            if (cb != NULL)
            {
                /* --- Determine trigger type from IRQC bits --- */
                uint32_t irqc = (PORTx->PCR[pin] & PORT_PCR_IRQC_MASK) >> PORT_PCR_IRQC_SHIFT;
                uint32_t event = HAL_GPIO_TRIGGER_FALLING_EDGE;

                switch (irqc)
                {
                    case PORT_INT_RISING_EDGE:
                        event = HAL_GPIO_TRIGGER_RISING_EDGE;
                        break;
                    case PORT_INT_FALLING_EDGE:
                        event = HAL_GPIO_TRIGGER_FALLING_EDGE;
                        break;
                    case PORT_INT_EITHER_EDGE:
                        event = HAL_GPIO_TRIGGER_EITHER_EDGE;
                        break;
                    default:
                        event = HAL_GPIO_TRIGGER_NONE;
                        break;
                }

                cb(pin_index, event);
            }
        }
    }
}

/* Individual IRQ handlers for each PORT, calling the common handler */
void PORTA_IRQHandler(void)
{
    PORT_IRQHandler(0);
}
void PORTB_IRQHandler(void)
{
    PORT_IRQHandler(1);
}
void PORTC_IRQHandler(void)
{
    PORT_IRQHandler(2);
}
void PORTD_IRQHandler(void)
{
    PORT_IRQHandler(3);
}
void PORTE_IRQHandler(void)
{
    PORT_IRQHandler(4);
}
