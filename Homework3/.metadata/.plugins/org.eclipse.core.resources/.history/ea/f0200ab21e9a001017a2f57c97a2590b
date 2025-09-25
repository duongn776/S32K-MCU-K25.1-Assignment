/*
 * Driver_PORT.h
 *
 *  Created on: Sep 21, 2025
 *      Author: nhduo
 */

#ifndef INC_DRIVER_PORT_H_
#define INC_DRIVER_PORT_H_

#include "S32K144.h"
#include "HWAccess_PORT.h"
/* Alias for PORTx */
#define PORTA   IP_PORTA
#define PORTB   IP_PORTB
#define PORTC   IP_PORTC
#define PORTD   IP_PORTD
#define PORTE   IP_PORTE

/* Alias for PCC */
#define PCC		IP_PCC

#define PORT_OUTPUT 1
#define PORT_INPUT  0

#define ENABLE 	1
#define DISABLE 0
/**
  * @brief  PORT Init structure definition
  */
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

/** @defgroup PORT_pins_define  PORT pins define
  * @{
  */
#define PORT_PIN_0      (0U)
#define PORT_PIN_1      (1U)
#define PORT_PIN_2      (2U)
#define PORT_PIN_3      (3U)
#define PORT_PIN_4      (4U)
#define PORT_PIN_5      (5U)
#define PORT_PIN_6      (6U)
#define PORT_PIN_7      (7U)
#define PORT_PIN_8      (8U)
#define PORT_PIN_9      (9U)
#define PORT_PIN_10     (10U)
#define PORT_PIN_11     (11U)
#define PORT_PIN_12     (12U)
#define PORT_PIN_13     (13U)
#define PORT_PIN_14     (14U)
#define PORT_PIN_15     (15U)
#define PORT_PIN_16     (16U)
#define PORT_PIN_17     (17U)
#define PORT_PIN_18     (18U)
#define PORT_PIN_19     (19U)
#define PORT_PIN_20     (20U)
#define PORT_PIN_21     (21U)
#define PORT_PIN_22     (22U)
#define PORT_PIN_23     (23U)
#define PORT_PIN_24     (24U)
#define PORT_PIN_25     (25U)
#define PORT_PIN_26     (26U)
#define PORT_PIN_27     (27U)
#define PORT_PIN_28     (28U)
#define PORT_PIN_29     (29U)
#define PORT_PIN_30     (30U)
#define PORT_PIN_31     (31U)

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

/* API functions */
void PORT_Init(const PORT_Config_t *cfg);
void PORT_PeriClockControl(PORT_Type *base, uint8_t clockState);

#endif /* INC_DRIVER_PORT_H_ */
