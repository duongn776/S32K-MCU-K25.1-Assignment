/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    12/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   Simple User Application to test Bootloader jump functionality.
 *   The application performs:
 *   • Initialize GPIO for LED and buttons
 *   • Read button input from PC12
 *   • Control RED LED on PD16 based on button press
 *
 *   If Bootloader jumps successfully to this App:
 *     → The LED will toggle ON/OFF when pressing the button.
 ******************************************************************************/

#include "S32K144.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PCC        IP_PCC
#define PORTC      IP_PORTC
#define PORTD      IP_PORTD
#define PTC        IP_PTC
#define PTD        IP_PTD

#define LED_RED_PIN       15U      /* PD16 - Red LED */
#define BTN_PIN           12U      /* PC12 - Button input */

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static void GPIO_Init(void);
static void LED_Red_Toggle(void);
static void Button_LED_Control(void);

/*******************************************************************************
 * GPIO Initialization
 ******************************************************************************/
/**
 * @brief Initialize GPIO pins for LED and Button
 *        - Enable clock for PORTC, PORTD
 *        - Configure PC12 as input with pull-up
 *        - Configure PD16 as output for Red LED
 */
static void GPIO_Init(void)
{
    /* Enable clock for PORTC and PORTD */
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Configure PC12 as GPIO input with internal pull-up */
    PORTC->PCR[BTN_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    /* Configure PD16 as GPIO output for Red LED */
    PORTD->PCR[LED_RED_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= (1 << LED_RED_PIN);

    /* Turn off LED initially (LED OFF = logic HIGH) */
    PTD->PSOR = (1 << LED_RED_PIN);
}

/*******************************************************************************
 * LED Control Functions
 ******************************************************************************/

/**
 * @brief Toggle Red LED state
 */
static void LED_Red_Toggle(void)
{
    PTD->PTOR = (1 << LED_RED_PIN);    /* Toggle LED */
}

/*******************************************************************************
 * Button - LED Logic
 ******************************************************************************/
/**
 * @brief Check button state and control LED
 *        - Button pressed (low level) → toggle LED
 */
static void Button_LED_Control(void)
{
    /* Check if button is pressed (active low) */
        LED_Red_Toggle();

        /* Simple debounce delay */
        for (volatile uint32_t i = 0; i < 300000; i++);
}

/*******************************************************************************
 * Main Function
 ******************************************************************************/
int main(void)
{
    GPIO_Init();

    while (1)
    {
        Button_LED_Control();
    }
}
