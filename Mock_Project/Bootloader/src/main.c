/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    12/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   Bootloader application for S32K144 MCU.
 *   The bootloader performs the following:
 *   • If the button is pressed → enter Bootloader mode
 *   • If not pressed → jump to User Application (UserApp) located at 0xA000
 *
 *   Bootloader UART commands (future use):
 *   • Receive .SREC firmware file
 *   • Parse SREC file and flash data into the UserApp region
 ******************************************************************************/

#include "S32K144.h"
#include "Driver_USART.h"
#include "Driver_GPIO.h"
#include "Srec_Parser.h"
#include "Circular_Queue.h"
#include "Flash.h"
#include "s32_core_cm4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BTN1             PC13
#define APP_START_ADDR   0x0000A000U
#define UART_RX_BUF_SIZE 512

/* UART messages */
#define MSG_READY        "UART ready!!!\r\n"
#define MSG_BOOT         "Entering Bootloader mode...\r\n"
#define MSG_APP          "Jumping to UserApp...\r\n"
#define NUM_QUEUES 		 4
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_GPIO  Driver_GPIO0;
char uart_rx_char;
char current_line[QUEUE_MAX_LINE_LEN];
uint32_t line_index = 0;
SrecQueue_t srecQueues[NUM_QUEUES];
static volatile int curr_index = 0;
static volatile int process_index = 0;



/* Function pointer type for UserApp entry point */
typedef void (*AppEntry_t)(void);

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void UART_SendString(const char *s);
void SOSC_init_8MHz(void);
void SPLL_init_160MHz(void);
void UART_Init(void);
void Button_Init(void);
void Bootloader_Mode(void);
void JumpToUserApp(void);

/**
 * @brief Send a string over UART1
 *
 * @param s Pointer to the string to send
 * @return None
 */
void UART_SendString(const char *s)
{
    Driver_USART1.Send(s, myStrlen(s));
}

/**
 * @brief Initialize System Oscillator (SOSC) at 8 MHz external crystal
 *
 * @return None
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
 * @brief Initialize System PLL (SPLL) to generate 160 MHz system clock
 *
 * @return None
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
 * @brief Initialize push button on PC13 (active low)
 *
 * Configure GPIO pin PC13 as input with pull-up resistor.
 *
 * @return None
 */
void Button_Init(void)
{
    Driver_GPIO0.Setup(BTN1, NULL);
    Driver_GPIO0.SetDirection(BTN1, ARM_GPIO_INPUT);
    Driver_GPIO0.SetPullResistor(BTN1, ARM_GPIO_PULL_UP);
    Driver_GPIO0.SetEventTrigger(BTN1, ARM_GPIO_TRIGGER_NONE);
}

/**
 * @brief UART1 interrupt callback
 *
 * Called when UART1 receives data. Handles line buffering and pushes
 * complete SREC lines into circular queue.
 *
 * @param event UART event flags
 * @return None
 */
void UART1_Callback(uint32_t event)
{
    /* Check if the receive complete event flag is set */
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        /* If the received character is a newline or carriage return */
        if (uart_rx_char == '\r' || uart_rx_char == '\n')
        {
            /* Only push if we have collected characters in the current line */
            if (line_index > 0)
            {
                current_line[line_index] = '\0';
                Queue_Push(&srecQueues[curr_index], current_line);
                curr_index = (curr_index + 1) % NUM_QUEUES;
                line_index = 0;
            }
        }
        else
        {
            /* If not newline, append character to current line buffer */
            if (line_index < QUEUE_MAX_LINE_LEN - 1)
            {
                current_line[line_index++] = uart_rx_char;
            }
        }

        Driver_USART1.Receive(&uart_rx_char, 1);
    }
}


/**
 * @brief Initialize UART1 peripheral
 *
 * Configure UART1 with baud rate, data bits, parity, stop bits.
 *
 * @return None
 */
void UART_Init(void)
{
    /* Initialize USART1 for UART communication */
    Driver_USART1.Initialize(UART1_Callback);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8 |
                          ARM_USART_PARITY_NONE |
                          ARM_USART_STOP_BITS_1 |
                          ARM_USART_FLOW_CONTROL_NONE, 9600u);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1u);
    Driver_USART1.Control(ARM_USART_CONTROL_RX, 1u);
}

/**
 * @brief Handle a single SREC record
 *
 * Parse and program data from SREC record into flash memory.
 *
 * @param rec Pointer to SREC record structure
 * @return true if SREC END record detected, false otherwise
 */
uint8_t Bootloader_HandleSrecRecord(const SREC_Record *rec)
{
    uint8_t buf[8];        /* Temporary buffer for 8-byte flash programming */
    uint32_t addr;         /* Current flash address to program */
    uint8_t *pdata;        /* Pointer to record data */
    uint32_t len;          /* Remaining length of data to program */
    uint8_t chunk;         /* Size of current chunk (<= 8 bytes) */

    switch (rec->record_type)
    {
        case SREC_TYPE_S0:
            /* S0 record: header information only, no programming required */
            return false;

        case SREC_TYPE_S1:
        case SREC_TYPE_S2:
        case SREC_TYPE_S3:
        {
            /* S1/S2/S3 records: contain actual data to be programmed into flash */

            addr  = rec->address;             /* Starting address */
            pdata = (uint8_t*)rec->data;      /* Data pointer */
            len   = rec->data_length;         /* Data length */

            /* Program data into flash in 8-byte chunks */
            while (len > 0)
            {
                memset(buf, 0xFF, 8);         /* Fill buffer with default erased value */
                chunk = (len >= 8) ? 8 : len; /* Determine chunk size */
                memcpy(buf, pdata, chunk);    /* Copy data into buffer */

                DISABLE_INTERRUPTS();
                Program_LongWord_8B(addr, buf); /* Program 8 bytes into flash */
                ENABLE_INTERRUPTS();

                addr  += 8;
                pdata += 8;
                len   -= chunk;
            }

            return false;
        }

        case SREC_TYPE_S7:
        case SREC_TYPE_S8:
        case SREC_TYPE_S9:
            /* End-of-file records: indicate completion of SREC file */
            UART_SendString("SREC END DETECTED\r\n");
            return true;

        default:
            /* Unknown record type */
            UART_SendString("Unknown SREC record\r\n");
            return false;
    }
}


/**
 * @brief Enter Bootloader mode
 *
 * Initialize queues, receive SREC lines via UART, parse and flash them.
 *
 * @return None
 */
void Bootloader_Mode(void)
{
    uint8_t check;
    UART_SendString("\r\n=== BOOTLOADER MODE ===\r\n");
    UART_SendString("Send .SREC file via UART to update firmware.\r\n");

    /* Initialize all circular queues used for buffering SREC lines */
    for (int i = 0; i < NUM_QUEUES; i++)
    {
        Queue_Init(&srecQueues[i]);
    }

    Driver_USART1.Receive(&uart_rx_char, 1);

    while (1)
    {
        char srecLine[QUEUE_MAX_LINE_LEN];

        /* Try to pop one complete line from the current queue */
        if (Queue_Pop(&srecQueues[process_index], srecLine))
        {
            SREC_Record record;

            /* Parse the line into an SREC record structure */
            if (Srec_parse_line(srecLine, &record))
            {
                check = Bootloader_HandleSrecRecord(&record);

                if (check)
                {
                    UART_SendString("\r\n=== FINISHED ===\r\n");
                }
            }
        }

        /* Move to the next queue index (round-robin scheduling) */
        process_index = (process_index + 1) % NUM_QUEUES;
    }
}


/**
 * @brief Jump to User Application located at APP_START_ADDR
 *
 * Set stack pointer and reset vector, then jump to UserApp entry point.
 *
 * @return None
 */
void JumpToUserApp(void)
{
    UART_SendString("Jumping to User Application...\r\n");

    /* Read initial stack pointer value from the UserApp vector table */
    uint32_t appStack       = *((uint32_t *)APP_START_ADDR);

    /* Read reset vector (entry point) from the UserApp vector table */
    uint32_t appResetVector = *((uint32_t *)(APP_START_ADDR + 4U));

    /* Cast reset vector address to a function pointer */
    AppEntry_t appEntry     = (AppEntry_t)appResetVector;

    DISABLE_INTERRUPTS();

    /* Redirect the vector table to the UserApp region */
    SCB->VTOR = APP_START_ADDR;

    /* Set the Main Stack Pointer (MSP) to the UserApp stack value */
    __set_MSP(appStack);

    ENABLE_INTERRUPTS();

    /* Jump to the UserApp entry point */
    appEntry();

    while (1);
}


/**
 * @brief Main entry point
 *
 * Initialize system clocks, UART, button. If button pressed → Bootloader mode.
 * Otherwise → jump to User Application.
 *
 * @return int Always returns 0
 */
int main(void)
{
	static uint8_t app_erased = 0;
    /* Initialize system clock and peripherals */
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    UART_Init();
    Button_Init();

    UART_SendString(MSG_READY);
    Mem_43_INFLS_IPW_LoadAc();

    while (1)
    {
        if (Driver_GPIO0.GetInput(BTN1) == 0)
        {
            /* Button pressed → enter Bootloader mode */
        	if (!app_erased)
			{
				UART_SendString("Erasing APP area...\r\n");
				Erase_Multi_Sector(APP_START_ADDR, APP_SECTOR_COUNT);
				app_erased = 1;
				UART_SendString("Erase done.\r\n");
			}
            UART_SendString(MSG_BOOT);
            Bootloader_Mode();
        }
        else
        {
            /* Button not pressed → jump to User Application */
            UART_SendString(MSG_APP);
            JumpToUserApp();
        }
    }

    return 0;
}
