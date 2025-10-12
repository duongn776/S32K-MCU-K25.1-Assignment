/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    12/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   Use the APIs provided by the UART Driver and GPIO Driver to
 *   build an application to control LEDs as required:
 *   • Send “LED STATUS” to get the information of the LEDs and display it on the PC’s
 *   console.
 *   • Send “RED ON” to turn on the Red LED and “RED OFF” to turn off the Red LED.
 *   • Send “GREEN ON” to turn on the Green LED and “GREEN OFF” to turn off the Green LED.
 *   • Send “BLUE ON” to turn on the Blue LED and “BLUE OFF” to turn off the Blue LED
 *   • Send “HELP” to display the guideline.
• If the command is not in the correct format, print “Command not available”.
 ******************************************************************************/

#include "S32K144.h"
#include "Driver_USART.h"
#include "Driver_GPIO.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_RED     PD15
#define LED_GREEN   PD16
#define LED_BLUE    PD0
#define MAX_CMD_LEN 50

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_GPIO  Driver_GPIO0;
uint8_t rx_char;
char cmd_buffer[MAX_CMD_LEN];
uint32_t cmd_index = 0;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void USART_Callback(uint32_t event);
void Process_Command(char *cmd);
void UART_SendString(const char *s);
void LED_On(uint32_t pin);
void LED_Off(uint32_t pin);
uint8_t LED_Get(uint32_t pin);
void SOSC_init_8MHz(void);
void SPLL_init_160MHz(void);
void UART_Init(void);
void LED_Init(void);

/**
 * @brief Send a string over UART
 * 
 * @param s string to send
 */
void UART_SendString(const char *s)
{
    Driver_USART1.Send(s, myStrlen(s));
}

/**
 * @brief Turn on LED (active low)
 * 
 * @param pin pin number of the LED
 */
void LED_On(uint32_t pin)
{
    Driver_GPIO0.SetOutput(pin, 0u);
}

/**
 * @brief Turn off LED (active low)
 * 
 * @param pin pin number of the LED
 */
void LED_Off(uint32_t pin)
{
    Driver_GPIO0.SetOutput(pin, 1u);
}

/**
 * @brief Get LED status (active low)
 * 
 * @param pin pin number of the LED
 * @return uint8_t 1 if LED is ON, 0 if OFF
 */
uint8_t LED_Get(uint32_t pin)
{
    return !Driver_GPIO0.GetInput(pin);
}


/**
 * @brief Process received command
 * 
 * @param cmd command string
 * - HELP: show guidelines
 * - LED STATUS: show status of all LEDs
 * - RED ON / RED OFF: turn on/off red LED
 * - GREEN ON / GREEN OFF: turn on/off green LED
 * - BLUE ON / BLUE OFF: turn on/off blue LED
 * - Other: show "Command not available"
 */
void Process_Command(char *cmd)
{
    /* Process command */
   if (myStrcmp(cmd, "HELP") == 0)
   {
       UART_SendString("\r\nAvailable commands:\r\n");
       UART_SendString("  LED STATUS\r\n");
       UART_SendString("  RED ON / RED OFF\r\n");
       UART_SendString("  GREEN ON / GREEN OFF\r\n");
       UART_SendString("  BLUE ON / BLUE OFF\r\n");
   }
   else if (myStrcmp(cmd, "LED STATUS") == 0)
   {
       char msg[80];
       sprintf(msg, "\r\nRED:%s GREEN:%s BLUE:%s\r\n",
               LED_Get(LED_RED) ? "ON" : "OFF",
               LED_Get(LED_GREEN) ? "ON" : "OFF",
               LED_Get(LED_BLUE) ? "ON" : "OFF");
       UART_SendString(msg);
   }
   else if (myStrcmp(cmd, "RED ON") == 0)
   {
       LED_On(LED_RED);
       UART_SendString("\r\nRED LED ON\r\n");
   }
   else if (myStrcmp(cmd, "RED OFF") == 0)
   {
       LED_Off(LED_RED);
       UART_SendString("\r\nRED LED OFF\r\n");
   }
   else if (myStrcmp(cmd, "GREEN ON") == 0)
   {
       LED_On(LED_GREEN);
       UART_SendString("\r\nGREEN LED ON\r\n");
   }
   else if (myStrcmp(cmd, "GREEN OFF") == 0)
   {
       LED_Off(LED_GREEN);
       UART_SendString("\r\nGREEN LED OFF\r\n");
   }
   else if (myStrcmp(cmd, "BLUE ON") == 0)
   {
       LED_On(LED_BLUE);
       UART_SendString("\r\nBLUE LED ON\r\n");
   }
   else if (myStrcmp(cmd, "BLUE OFF") == 0)
   {
       LED_Off(LED_BLUE);
       UART_SendString("\r\nBLUE LED OFF\r\n");
   }
   else
   {
       UART_SendString("\r\nCommand not available\r\n");
   }
}

/**
 * @brief Callback function for USART events
 * 
 * @param event uart event
 * - ARM_USART_EVENT_RECEIVE_COMPLETE: a byte is received
 */
void USART_Callback(uint32_t event)
{
    /* Check for receive complete event */
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        char c = (char)rx_char;

        /* check for command end */
        if (c == '\r' || c == '\n')
        {
            cmd_buffer[cmd_index] = '\0';
            if (cmd_index > 0)
            {
                Process_Command(cmd_buffer);
                cmd_index = 0;
            }
            UART_SendString("\r\n> ");
        }
        else
        {
            /* Add character to command buffer */
            if (cmd_index < MAX_CMD_LEN - 1)
            {
                cmd_buffer[cmd_index++] = myToUpper(rx_char);
            }
        }

        /* Continue to receive next byte */
        Driver_USART1.Receive(&rx_char, 1);
    }
}

/**
 * @brief Initialize System Oscillator (SOSC) at 8 MHz external crystal
 * 
 */
void SOSC_init_8MHz(void)
{
    /* Enable clock SOSC */
    /* Wait for unlock */
    while (SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK);
    /* Disable SOSC to configure */
    SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK;

    /* Configure clock division by 1 */
    SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1) | SCG_SOSCDIV_SOSCDIV2(1);

    /* Configure crystal oscillator */
    SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2) |           /* Medium range (1–8 MHz) */
                   SCG_SOSCCFG_EREFS_MASK;          /* External crystal */

    /* Enable SOSC */
    SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK; 

    /* Wait until oscillator is stable */
    while (!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));
}


/**
 * @brief  Initialize System PLL (SPLL) to generate 160 MHz
 *         SPLL_CLK = SOSC_CLK * (MULT + 16) / ((PREDIV + 1) * 2)
 */
void SPLL_init_160MHz(void)
{
    /* Wait for unlock and disable to configure */
    while (SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK);
    SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK;

    /* Configure clock division
        SPLLDIV1 → divide core (DIV1=2)
        SPLLDIV2 → divide bus/flash (DIV2=3)
        => Core = 160 MHz / 2 = 80 MHz
        => Bus  = 160 MHz / 4 = 40 MHz
    */
    SCG->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(2) | SCG_SPLLDIV_SPLLDIV2(3);

    /* Configure PLL:
          MULT = 24 → (24 + 16) = 40
          PREDIV = 0 → divide 1
          => SPLL = 8 MHz * 40 / 2 = 160 MHz
    */
    SCG->SPLLCFG = SCG_SPLLCFG_PREDIV(0) | SCG_SPLLCFG_MULT(24);

    /* Enable PLL */
    SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN_MASK;

    /* Wait for PLL stable (lock) */
    while (!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK));
}

/**
 * @brief Initialize LEDs: Red, Green, Blue
 * 
 */
void LED_Init(void)
{
    /* Initialize Red LED */
    Driver_GPIO0.Setup(LED_RED, NULL);
    Driver_GPIO0.SetDirection(LED_RED, ARM_GPIO_OUTPUT);
    Driver_GPIO0.SetOutput(LED_RED, 1);

    /* Initialize Green LED */
    Driver_GPIO0.Setup(LED_GREEN, NULL);
    Driver_GPIO0.SetDirection(LED_GREEN, ARM_GPIO_OUTPUT);
    Driver_GPIO0.SetOutput(LED_GREEN, 1);

    /* Initialize Blue LED */
    Driver_GPIO0.Setup(LED_BLUE, NULL);
    Driver_GPIO0.SetDirection(LED_BLUE, ARM_GPIO_OUTPUT);
    Driver_GPIO0.SetOutput(LED_BLUE, 1);
}

void UART_Init(void)
{
    /* Initialize USART1 for UART communication */
    Driver_USART1.Initialize(USART_Callback);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8 |
                          ARM_USART_PARITY_NONE |
                          ARM_USART_STOP_BITS_1 |
                          ARM_USART_FLOW_CONTROL_NONE, 115200u);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1u);
    Driver_USART1.Control(ARM_USART_CONTROL_RX, 1u);
}

/**
 * @brief Main function: program entry point turn on/off LEDs via UART commands
 * 
 * @return int 
 */
int main(void)
{
    /* Configure system clock */
    SOSC_init_8MHz();
    SPLL_init_160MHz();

    /* Initialize LEDs */
    LED_Init();

    /* Initialize USART1 for UART communication */
   UART_Init();

   /* Send initial message */
   UART_SendString("UART ready!!! Type command:\r\n> ");

    /* Start receiving 1 byte */
    Driver_USART1.Receive(&rx_char, 1);

    while (1);

    return 0;
}
