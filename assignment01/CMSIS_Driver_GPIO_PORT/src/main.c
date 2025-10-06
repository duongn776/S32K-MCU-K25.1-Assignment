/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    04/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   Toggle LED when button is pressed using CMSIS Driver GPIO interrupt
 ******************************************************************************/

#include "Driver_GPIO.h"
#include "core_cm4.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_RED     	 PD16
#define LED_GREEN   	 PD15
#define BTN1        	 PC12
#define BTN2        	 PC13
#define BUTTON_PRESSED   0U
#define BUTTON_RELEASE   1U

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern ARM_DRIVER_GPIO Driver_GPIO0;
volatile uint8_t g_button1 = BUTTON_RELEASE;
volatile uint8_t g_button2 = BUTTON_RELEASE;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void GPIO_ApplicationEventCallback(uint32_t pin, uint32_t event);
void delay(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief GPIO Application Event Callback: called automatically from PORT_IRQHandler()
 *
 * @param pin   Pin that triggered the event
 * @param event Event type (RISING/FALLING/EITHER)
 */
void GPIO_ApplicationEventCallback(uint32_t pin, uint32_t event)
{
    if (event == ARM_GPIO_EVENT_FALLING_EDGE)
    {
        if (pin == BTN1)
        {
            g_button1 = BUTTON_PRESSED;
        }
        else if (pin == BTN2)
        {
            g_button2 = BUTTON_PRESSED;
        }
    }
}

/**
 * @brief Simple delay (software loop)
 */
void delay(void)
{
    for (volatile uint32_t i = 0; i < 100000; i++);
}

/**
 * @brief Main function
 */
int main(void)
{
    /* ==================== LED Configuration ==================== */
    Driver_GPIO0.Setup(LED_RED, NULL);
    Driver_GPIO0.SetDirection(LED_RED, ARM_GPIO_OUTPUT);
    Driver_GPIO0.SetOutput(LED_RED, 1);

    Driver_GPIO0.Setup(LED_GREEN, NULL);
    Driver_GPIO0.SetDirection(LED_GREEN, ARM_GPIO_OUTPUT);
    Driver_GPIO0.SetOutput(LED_GREEN, 1);

    /* ==================== Button Configuration ==================== */
    Driver_GPIO0.Setup(BTN1, GPIO_ApplicationEventCallback);
    Driver_GPIO0.SetDirection(BTN1, ARM_GPIO_INPUT);
    Driver_GPIO0.SetPullResistor(BTN1, ARM_GPIO_PULL_UP);
    Driver_GPIO0.SetEventTrigger(BTN1, ARM_GPIO_TRIGGER_FALLING_EDGE);

    Driver_GPIO0.Setup(BTN2, GPIO_ApplicationEventCallback);
    Driver_GPIO0.SetDirection(BTN2, ARM_GPIO_INPUT);
    Driver_GPIO0.SetPullResistor(BTN2, ARM_GPIO_PULL_UP);
    Driver_GPIO0.SetEventTrigger(BTN2, ARM_GPIO_TRIGGER_FALLING_EDGE);

    /* ==================== NVIC Configuration ==================== */
    NVIC_ClearPendingIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTC_IRQn, 2);
    NVIC_EnableIRQ(PORTC_IRQn);

    while (1)
    {
        if (g_button1 == BUTTON_PRESSED)
        {
            g_button1 = BUTTON_RELEASE;
            uint32_t state = Driver_GPIO0.GetInput(LED_RED);
            Driver_GPIO0.SetOutput(LED_RED, !state);
        }

        if (g_button2 == BUTTON_PRESSED)
        {
            g_button2 = BUTTON_RELEASE;
            uint32_t state = Driver_GPIO0.GetInput(LED_GREEN);
            Driver_GPIO0.SetOutput(LED_GREEN, !state);
        }

        delay();
    }
}

