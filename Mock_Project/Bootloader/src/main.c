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
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BTN1             PC13
#define APP_START_ADDR   0x0000A000U
#define UART_RX_BUF_SIZE 2048

/* UART messages */
#define MSG_READY        "UART ready!!!\r\n"
#define MSG_BOOT         "Entering Bootloader mode...\r\n"
#define MSG_APP          "Jumping to UserApp...\r\n"
#define NUM_QUEUES 4
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
typedef enum
{
    Q_IDLE = 0,
    Q_WRITING,
    Q_READY,
    Q_READING
} QueueState_t;
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_GPIO  Driver_GPIO0;
char uart_rx_char;
char current_line[QUEUE_MAX_LINE_LEN];
uint32_t line_index = 0;
SrecQueue_t srecQueues[NUM_QUEUES];
volatile uint8_t uart_rx_buffer[UART_RX_BUF_SIZE];
volatile uint16_t uart_rx_head = 0;
volatile uint16_t uart_rx_tail = 0;
static volatile QueueState_t qstate[NUM_QUEUES];
static volatile uint8_t activeQueueIndex = 0;
static volatile uint8_t processQueueIndex = 0;




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
 * @brief Send a string over UART
 *
 * @param s string to send
 */
void UART_SendString(const char *s)
{
    Driver_USART1.Send(s, myStrlen(s));
}

/*******************************************************************************
 * Clock Configuration
 ******************************************************************************/
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

/*******************************************************************************
 * Peripheral Initialization
 ******************************************************************************/
/**
 * @brief Initialize push button on PC13 (active low)
 */
void Button_Init(void)
{
    Driver_GPIO0.Setup(BTN1, NULL);
    Driver_GPIO0.SetDirection(BTN1, ARM_GPIO_INPUT);
    Driver_GPIO0.SetPullResistor(BTN1, ARM_GPIO_PULL_UP);
    Driver_GPIO0.SetEventTrigger(BTN1, ARM_GPIO_TRIGGER_NONE);
}

void UART1_Callback(uint32_t event)
{
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        uint16_t next = (uart_rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != uart_rx_tail)
        {
            uart_rx_buffer[uart_rx_head] = uart_rx_char;
            uart_rx_head = next;
        }
        Driver_USART1.Receive(&uart_rx_char, 1);
    }
}

/**
 * @brief Initialize UART1 at 115200 baud rate
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



void Process_UART_Buffer(void)
{
	uint8_t next = 0;
	char c;
    while (uart_rx_tail != uart_rx_head)
    {
        c = uart_rx_buffer[uart_rx_tail];

        if (c == '\r')
        {
        	uart_rx_tail = (uart_rx_tail + 2) % UART_RX_BUF_SIZE;
            if (line_index > 0)
            {
                current_line[line_index] = '\0';

                /* Push the line into the current queue */
                if (Queue_Push(&srecQueues[activeQueueIndex], current_line))
                {
                    /* Mark queue ready for parser */
                    qstate[activeQueueIndex] = Q_READY;

                    /* Find a new queue to receive the next line */
                    next = (activeQueueIndex + 1) % NUM_QUEUES;
                    for (int i = 0; i < NUM_QUEUES; i++)
                    {
                        if (qstate[next] == Q_IDLE)
                        {
                            qstate[next] = Q_WRITING;
                            activeQueueIndex = next;
                            i = NUM_QUEUES;
                        }
                        next = (next + 1) % NUM_QUEUES;
                    }
                }
                line_index = 0;
            }
        }
        else
        {
            if (line_index < QUEUE_MAX_LINE_LEN - 1)
            {
                current_line[line_index++] = c;
            }
            uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUF_SIZE;
        }
    }
}

uint8_t Bootloader_HandleSrecRecord(const SREC_Record *rec)
{
	static uint8_t app_erased = 0;

    switch (rec->record_type)
    {
        case SREC_TYPE_S0:
            /* Header file */
        	return false;

        case SREC_TYPE_S1:
        case SREC_TYPE_S2:
        case SREC_TYPE_S3:
        {
        	if (!app_erased)
        	{
        	    UART_SendString("Erasing APP area...\r\n");
        	    //Erase_Multi_Sector(APP_START_ADDR, APP_SECTOR_COUNT);
        	    app_erased = 1;
        	    UART_SendString("Erase done.\r\n");
        	}
            uint32_t addr  = rec->address;
            uint8_t *pdata = (uint8_t*)rec->data;
            uint32_t len   = rec->data_length;

            while (len > 0)
            {
                uint8_t buf[8];
                memset(buf, 0xFF, 8);

                uint8_t chunk = (len >= 8) ? 8 : len;
                memcpy(buf, pdata, chunk);

                if (!Program_LongWord_8B(addr, buf))
                {
                    UART_SendString("FLASH WRITE ERROR\r\n");
                    //break;
                }

                addr  += 8;
                pdata += 8;
                len   -= chunk;
            }

            return false;
        }

        case SREC_TYPE_S7:
        case SREC_TYPE_S8:
        case SREC_TYPE_S9:
        	/* Only signals that the SREC file has ended */
            UART_SendString("SREC END DETECTED\r\n");
            return true;

        default:
            UART_SendString("Unknown SREC record\r\n");
            return false;
    }
}


void Bootloader_Mode(void)
{
    UART_SendString("\r\n=== BOOTLOADER MODE ===\r\n");
    UART_SendString("\r\n=== Send file SREC ===\r\n");
    for (int i = 0; i < NUM_QUEUES; i++)
    {
        Queue_Init(&srecQueues[i]);
        qstate[i] = Q_IDLE;
    }

    activeQueueIndex = 0;
    qstate[0] = Q_WRITING;
    processQueueIndex = 0;
    Driver_USART1.Receive(&uart_rx_char, 1);
    while (1)
    {
        /* receive new data from buffer */
        Process_UART_Buffer();

        /* Parser processes queue ready */
        if (qstate[processQueueIndex] == Q_READY)
        {
            qstate[processQueueIndex] = Q_READING;

            SrecQueue_t *q = &srecQueues[processQueueIndex];
            char line[QUEUE_MAX_LINE_LEN];

            while (0)
            {
                Queue_Pop(q, line);
                SREC_Record rec;
                if (Srec_parse_line(line, &rec))
                {
                    uint8_t end_flag = Bootloader_HandleSrecRecord(&rec);

                    if (end_flag)
                    {
                        UART_SendString("Bootloader: Parsing completed. STOP.\r\n");
                        //break;
                    }
                }
                else
                {
                    UART_SendString("Parse ERROR\r\n");
                }

            }
            qstate[processQueueIndex] = Q_IDLE;
        }

        processQueueIndex = (processQueueIndex + 1) % NUM_QUEUES;
    }
}



/**
 * @brief Jump to User Application located at APP_START_ADDR
 */
void JumpToUserApp(void)
{
    UART_SendString("Jumping to User Application...\r\n");

    uint32_t appStack       = *((uint32_t *)APP_START_ADDR);
    uint32_t appResetVector = *((uint32_t *)(APP_START_ADDR + 4U));
    AppEntry_t appEntry     = (AppEntry_t)appResetVector;

    /* Set vector table offset to the start of UserApp */
    SCB->VTOR = APP_START_ADDR;

    /* Set Main Stack Pointer */
    __set_MSP(appStack);

    /* Jump to Application Reset Handler */
    appEntry();

    while (1);
}

/*******************************************************************************
 * Main Function
 ******************************************************************************/
int main(void)
{
    /* Initialize system clock and peripherals */
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    UART_Init();
    Button_Init();

    UART_SendString(MSG_READY);


    while (1)
    {
        if (Driver_GPIO0.GetInput(BTN1) == 0)
        {
            /* Button pressed → enter Bootloader mode */
            //UART_SendString(MSG_BOOT);
            Bootloader_Mode();
        }
        else
        {
            /* Button not pressed → jump to User Application */
            //UART_SendString(MSG_APP);
        	JumpToUserApp();
        }



    }

    return 0;
}

void HardFault_Handler(void)
{
	UART_SendString("Jumping to HardFault...\r\n");
}
