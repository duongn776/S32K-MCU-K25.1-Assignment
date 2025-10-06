/**
 ******************************************************************************
 * @file    HAL_GPIO.h
 * @author  Nguyen Hoang Duong
 * @date    21-Sep-2025
 * @brief   Hardware Abstraction Layer for GPIO and PORT on NXP S32K144 MCU
 *
 * @note    Provides HAL-level APIs for GPIO initialization, pin control,
 *          pull-up/down, and interrupt configuration using CMSIS-compliant style.
 ******************************************************************************
 */
#ifndef INC_HAL_GPIO_H_
#define INC_HAL_GPIO_H_

#include "HAL_Common.h"

/* ===========================================================
 *                      GLOBAL DEFINES
 * =========================================================== */

/* --- Alias for PORT base addresses --- */
#define PORTA   IP_PORTA
#define PORTB   IP_PORTB
#define PORTC   IP_PORTC
#define PORTD   IP_PORTD
#define PORTE   IP_PORTE


/* --- Alias for PCC base address --- */
#define PCC		IP_PCC

/* --- Generic defines --- */
#define PORT_OUTPUT 1
#define PORT_INPUT  0
#define ENABLE      1
#define DISABLE     0

/**
 * @brief GPIO callback function prototype
 */
typedef void (*HAL_GPIO_SignalEvent_t) (uint32_t pin, uint32_t event);

/**
\brief HAL_GPIO Event Trigger
*/
typedef enum {
  HAL_GPIO_TRIGGER_NONE,                ///< None (default)
  HAL_GPIO_TRIGGER_RISING_EDGE,         ///< Rising-edge
  HAL_GPIO_TRIGGER_FALLING_EDGE,        ///< Falling-edge
  HAL_GPIO_TRIGGER_EITHER_EDGE          ///< Either edge (rising and falling)
} HAL_GPIO_EVENT_TRIGGER;

typedef struct
{
    PORT_Type *portBase;   /*!< PORTx base (PORTA..PORTE) */

    uint32_t   pin;        /*!< Specifies the pin muxing.
                        This parameter can be a value of @ref PORT_pin_define */

    uint32_t   mux;        /*!< Specifies the pin muxing.
                                This parameter can be a value of @ref PORT_mux_define */

    uint32_t   pull;       /*!< Specifies Pull-Up or Pull-Down.
                                This parameter can be a value of @ref PORT_pull_define */
    uint32_t   interrupt;  /*!< Specifies the pin interrupt.
                                This parameter can be a value of @ref PORT_interrupts_define */
} PORT_Config_t;

/** @defgroup PORT_mux_define  PORT mux define
  * @{
  */
#define PORT_MUX_DISABLED             (0U)
#define PORT_MUX_GPIO                 (1U)
#define PORT_MUX_ALT2                 (2U)
#define PORT_MUX_ALT3                 (3U)
#define PORT_MUX_ALT4                 (4U)
#define PORT_MUX_ALT5                 (5U)
#define PORT_MUX_ALT6                 (6U)
#define PORT_MUX_ALT7                 (7U)
/**
  * @}
  */

/** @defgroup PORT_pull_define PORT pull define
  * @brief PORT Pull-Up or Pull-Down Activation
  * @{
  */
#define PORT_NOPULL        (0)   /*!< No Pull-up or Pull-down activation  */
#define PORT_PULLUP        (1)   /*!< Pull-up activation                  */
#define PORT_PULLDOWN      (2)   /*!< Pull-down activation                */
/**
  * @}
  */


/** @defgroup PORT_interrupts_define  PORT interrupts define
  * @{
  */
#define PORT_INT_DISABLED      (0b0000U)
#define PORT_INT_DMA_RISING    (0b0001U)
#define PORT_INT_DMA_FALLING   (0b0010U)
#define PORT_INT_DMA_EITHER    (0b0011U)
#define PORT_INT_LOGIC_ZERO    (0b1000U)
#define PORT_INT_RISING_EDGE   (0b1001U)
#define PORT_INT_FALLING_EDGE  (0b1010U)
#define PORT_INT_EITHER_EDGE   (0b1011U)
#define PORT_INT_LOGIC_ONE     (0b1100U)

/**
  * @}
  */

/* ===========================================================
 *                     API FUNCTION
 * =========================================================== */

/* --- PORT-level HAL functions --- */
void PORT_Init(const PORT_Config_t *cfg);
void PORT_PeriClockControl(PORT_Type *base, uint8_t clockState);
void PORT_NoPull(PORT_Type *base, uint8_t pin);
void PORT_PullUp(PORT_Type *base, uint8_t pin);
void PORT_PullDown(PORT_Type *base, uint8_t pin);
void PORT_SetInterruptConfig(PORT_Type *base, uint8_t pin, uint8_t irqc);
void PORT_ClearInterruptConfig(PORT_Type *base, uint8_t pin);
uint32_t PORT_GetInterruptFlags(PORT_Type *base);
void PORT_ClearInterruptFlag(PORT_Type *base, uint8_t pin);

/* --- GPIO-level HAL functions --- */
void GPIO_SetPin(GPIO_Type *pGPIOx, uint8_t pin);
void GPIO_ClearPin(GPIO_Type *pGPIOx, uint8_t pin);
void GPIO_TogglePin(GPIO_Type *pGPIOx, uint8_t pin);
void GPIO_WritePin(GPIO_Type *pGPIOx, uint8_t pin, uint8_t val);
uint8_t GPIO_ReadPin(GPIO_Type *pGPIOx, uint8_t pin);
void GPIO_SetPinDirection(GPIO_Type *pGPIOx, uint8_t pin, uint8_t dir);
void GPIO_Init(uint32_t pin, HAL_GPIO_SignalEvent_t cb_event);

/* --- Interrupt handler (shared for all ports) --- */
void PORT_IRQHandler(uint8_t port_index);
#endif /* INC_HAL_GPIO_H_ */
