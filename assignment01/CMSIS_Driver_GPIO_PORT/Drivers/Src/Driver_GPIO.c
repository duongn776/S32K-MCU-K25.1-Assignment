/*
 * Driver_GPIO.c
 *
 *  Created on: Sep 21, 2025
 *      Author: nhduong
 */

#include "Driver_GPIO.h"

// Pin mapping
#define GPIO_MAX_PORTS          5U
#define GPIO_MAX_PINS           (GPIO_MAX_PORTS * 32U)   // 160

#define PIN_IS_AVAILABLE(n)     ((n) < GPIO_MAX_PINS)

#define GET_PORT(n)             ((n) / 32)   // Lấy chỉ số port (0=A, 1=B, 2=C, ...)
#define GET_PIN(n)              ((n) % 32)   // Lấy chỉ số pin trong port

static PORT_Type* const PORT_BASE[] = IP_PORT_BASE_PTRS;
static GPIO_Type* const GPIO_BASE[] = IP_GPIO_BASE_PTRS;

static ARM_GPIO_SignalEvent_t GPIO_Callbacks[GPIO_MAX_PINS] = {0};

/**
 * @brief The function ARM_GPIO_Setup sets-up the specified pin as GPIO with default configuration.
 *        Pin is configured as input without pull-resistor and without event trigger.
 *
 * @param pin GPIO pin number
 * @param cb_event Callback function for GPIO events
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure.
 */
static int32_t GPIO_Setup (ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event) {
    int32_t result = ARM_DRIVER_OK;

    if (PIN_IS_AVAILABLE(pin)) {
        uint8_t port_index = GET_PORT(pin);
        uint8_t pin_num    = GET_PIN(pin);

        /* save callback */
        GPIO_Callbacks[pin] = cb_event;

        /* Enable for PORTx */
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
    else {
        result = ARM_GPIO_ERROR_PIN;
    }

    return result;
}


/**
 * @brief The function ARM_GPIO_SetDirection configures the direction of the specified pin.
 *
 * @param pin The pin number to configure (0-31).
 * @param direction The desired pin direction (input/output).
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure.
 */
static int32_t GPIO_SetDirection (ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction) {
  int32_t result = ARM_DRIVER_OK;


  if (PIN_IS_AVAILABLE(pin)) {
	  uint8_t port_index = GET_PORT(pin);
	  uint8_t pin_num = GET_PIN(pin);

    switch (direction) {
      case ARM_GPIO_INPUT:
    	  GPIO_SetPinDirection(GPIO_BASE[port_index], pin_num, 0);
        break;
      case ARM_GPIO_OUTPUT:
    	  GPIO_SetPinDirection(GPIO_BASE[port_index], pin_num, 1);
        break;
      default:
        result = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
  } else {
    result = ARM_GPIO_ERROR_PIN;
  }

  return result;
}

/**
 * @brief The function ARM_GPIO_SetOutputMode configures the output mode of the specified pin.
 *
 * @param pin GPIO pin number
 * @param mode Desired output mode (push-pull/open-drain)
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure.
 */
static int32_t GPIO_SetOutputMode(ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode) {
    int32_t result = ARM_DRIVER_OK;

    if (PIN_IS_AVAILABLE(pin)) {
        switch (mode) {
            case ARM_GPIO_PUSH_PULL:
                /* S32K144 GPIO default is push-pull, nothing to configure */
                break;

            case ARM_GPIO_OPEN_DRAIN:
                result = ARM_DRIVER_ERROR_UNSUPPORTED;
                break;

            default:
                result = ARM_DRIVER_ERROR_PARAMETER;
                break;
        }
    } else {
        result = ARM_GPIO_ERROR_PIN;
    }

    return result;
}


/**
 * @brief The function ARM_GPIO_SetPullResistor configures the pull resistor of the specified pin.
 *
 * @param pin GPIO pin number
 * @param resistor Desired pull resistor configuration (none/up/down)
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure.
 */
static int32_t GPIO_SetPullResistor (ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor) {
  int32_t result = ARM_DRIVER_OK;

  if (PIN_IS_AVAILABLE(pin)) {
	  uint8_t port_index = GET_PORT(pin);
	  uint8_t pin_num    = GET_PIN(pin);


    switch (resistor) {
      case ARM_GPIO_PULL_NONE:
    	  PORT_NoPull(PORT_BASE[port_index], pin_num);
        break;
      case ARM_GPIO_PULL_UP:
    	  PORT_PullUp(PORT_BASE[port_index], pin_num);
        break;
      case ARM_GPIO_PULL_DOWN:
    	  PORT_PullDown(PORT_BASE[port_index], pin_num);
        break;
      default:
        result = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
  } else {
    result = ARM_GPIO_ERROR_PIN;
  }

  return result;
}

/**
 * @brief The function ARM_GPIO_SetEventTrigger configures the event trigger of the specified pin.
 *
 * @param pin GPIO pin number
 * @param trigger Desired event trigger configuration (none/rising/falling/either)
 * @return int32_t ARM_DRIVER_OK on success, or an error code on failure.
 */
static int32_t GPIO_SetEventTrigger (ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger) {
  int32_t result = ARM_DRIVER_OK;

  if (PIN_IS_AVAILABLE(pin)) {
	  uint8_t port_index = GET_PORT(pin);
	  uint8_t pin_num    = GET_PIN(pin);
    switch (trigger) {
      case ARM_GPIO_TRIGGER_NONE:
    	  PORT_ClearInterruptConfig(PORT_BASE[port_index], pin_num);
        break;
      case ARM_GPIO_TRIGGER_RISING_EDGE:
    	  PORT_SetInterruptConfig(PORT_BASE[port_index], pin_num, PORT_INT_RISING_EDGE);
        break;
      case ARM_GPIO_TRIGGER_FALLING_EDGE:
    	  PORT_SetInterruptConfig(PORT_BASE[port_index], pin_num, PORT_INT_FALLING_EDGE);
        break;
      case ARM_GPIO_TRIGGER_EITHER_EDGE:
    	  PORT_SetInterruptConfig(PORT_BASE[port_index], pin_num, PORT_INT_EITHER_EDGE);
        break;
      default:
        result = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
  } else {
    result = ARM_GPIO_ERROR_PIN;
  }

  return result;
}

/**
 * @brief The function ARM_GPIO_SetOutput sets the level of the specified pin defined as output to the value specified by val.
 *
 * @param pin GPIO pin number
 * @param val Desired output level (0 or 1)
 */
static void GPIO_SetOutput (ARM_GPIO_Pin_t pin, uint32_t val) {

  if (PIN_IS_AVAILABLE(pin)) {
	  uint8_t port_index = GET_PORT(pin);
	  uint8_t pin_num    = GET_PIN(pin);
	  GPIO_WritePin(GPIO_BASE[port_index], pin_num, val);
  }
}

/**
 * @brief The function ARM_GPIO_GetInput reads the level of the specified pin.
 *
 * @param pin GPIO pin number
 * @param val Pointer to store the read value (0 or 1)
 */
static uint32_t GPIO_GetInput (ARM_GPIO_Pin_t pin) {
  uint32_t val = 0U;

  if (PIN_IS_AVAILABLE(pin)) {
	 uint8_t port_index = GET_PORT(pin);
	 uint8_t pin_num    = GET_PIN(pin);
	 val = GPIO_ReadPin(GPIO_BASE[port_index], pin_num);
  }
  return val;
}


/**
 * @brief GPIO Driver access structure
 */
ARM_DRIVER_GPIO Driver_GPIO0 = {
  GPIO_Setup,
  GPIO_SetDirection,
  GPIO_SetOutputMode,
  GPIO_SetPullResistor,
  GPIO_SetEventTrigger,
  GPIO_SetOutput,
  GPIO_GetInput
};
