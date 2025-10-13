/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    12/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   - Use LPIT0 periodic interrupt (3s) to toggle RGB LEDs sequentially.
 *   - LED sequence: RED -> GREEN -> BLUE -> repeat.
 ******************************************************************************/

#include "S32K144.h"
#include "core_cm4.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PCC   IP_PCC
#define LPIT0 IP_LPIT0
#define PORTC IP_PORTC
#define PORTD IP_PORTD
#define PTD   IP_PTD
#define SCG   IP_SCG
#define LED_RED    15
#define LED_GREEN  16
#define LED_BLUE   0

/*******************************************************************************
 * Variables
 ******************************************************************************/
typedef enum 
{
    RED = 0, 
    GREEN = 1, 
    BLUE = 2
} led_color_t;
volatile led_color_t led_state = RED;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void SOSC_init_8MHz(void);
void SPLL_init_160MHz(void);
void GPIO_init(void);
void LPIT0_Init(void);

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
 * @brief Initialize GPIO pins for RGB LED
 * 
 */
void GPIO_init(void)
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Set pin mux to GPIO */
    PORTD->PCR[LED_RED]   = PORT_PCR_MUX(1);
    PORTD->PCR[LED_GREEN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_BLUE]  = PORT_PCR_MUX(1);

    /* Set pins as output */
    PTD->PDDR |= (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);

    /* Turn off all LEDs (active low) */
    PTD->PSOR = (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);
}

/**
 * @brief Initialize LPIT0 for periodic interrupts
 * 
 */
void LPIT0_Init(void)
{
    /* 1. Enable PCC clock for LPIT0 */
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    /* 2. Enable LPIT module clock */
    LPIT0->MCR = LPIT_MCR_M_CEN_MASK;

    /* 3. Wait for sync */
    for (volatile int i = 0; i < 100; i++);

    /* 4. Configure periodic mode (MODE=0) */
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_MODE(0);

    /* 5. Set timeout value */
    LPIT0->TMR[0].TVAL = 120000000 - 1; // 3s

    /* 6. Enable interrupt */
    LPIT0->MIER = LPIT_MIER_TIE0_MASK;
    NVIC_EnableIRQ(LPIT0_Ch0_IRQn);

    /* 7. Optional low power settings */
    LPIT0->MCR |= LPIT_MCR_DBG_EN_MASK | LPIT_MCR_DOZE_EN_MASK;

    /* 8. Enable timer */
    LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
}

/**
 * @brief LPIT0 Channel 0 Interrupt Handler
 * 
 */
void LPIT0_Ch0_IRQHandler(void)
{
    /* Clear interrupt flag */
    LPIT0->MSR = LPIT_MSR_TIF0_MASK;

    /* Turn off all LEDs */
    PTD->PSOR = (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);

    /* Turn on next LED */
    switch (led_state)
    {
        case RED:
        	PTD->PCOR = (1 << LED_RED);
        	break;
        case GREEN:
        	PTD->PCOR = (1 << LED_GREEN);
        	break;
        case BLUE:
        	PTD->PCOR = (1 << LED_BLUE);
        	break;
        default:
        	/* Nothing to do */
        	break;
    }

    /* Next LED in sequence */
    led_state = (led_state + 1) % 3;
}

/**
 * @brief Main function: Control RGB LED based on LPIT0 interrupts every 3 seconds
 * 
 * @return int 
 */
int main(void)
{
	/* Configure system clock */
	SOSC_init_8MHz();
	SPLL_init_160MHz();

    /* Initialize peripherals */
	GPIO_init();
    LPIT0_Init();

    while (1)
    {
        __WFI(); // Wait for interrupt (low power)
    }
}
