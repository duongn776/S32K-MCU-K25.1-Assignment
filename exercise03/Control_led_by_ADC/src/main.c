/******************************************************************************
 * File:    main.c
 * Author:  Nguyen Hoang Duong
 * Date:    04/10/2025
 * Target:  NXP S32K144 MCU
 * Tool:    S32 Design Studio
 *
 * Description:
 *   - Read value from ADC potentiometer (PTC14 - ADC0_SE12).
 *   - Scale ADC value (0–4095) to voltage 0–5000 mV.
 *   - Control RGB LED (Port D: RED = PTD15, GREEN = PTD16, BLUE = PTD0)
 *     based on voltage thresholds:
 *        + 0    – 1249 mV: Turn off LED
 *        + 1250 – 2499 mV: Turn on blue LED
 *        + 2500 – 3749 mV: Turn on green LED
 *        + 3750 – 5000 mV: Turn on red LED
 ******************************************************************************/

#include "S32K144.h"  

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PCC   IP_PCC
#define ADC0  IP_ADC0
#define PORTC IP_PORTC
#define PORTD IP_PORTD
#define PTD   IP_PTD
#define SCG   IP_SCG    
#define LED_RED    15
#define LED_GREEN  16
#define LED_BLUE   0
#define ADC_MAX  4095  
#define SCALE_MV 5000 


void ADC0_Init(void)
{
    /* Enable FIRC if is not already enabled */
    if (!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)) 
    {
        /* Enable FIRC */
        SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN_MASK;
        /* Wait until FIRC is stable */
        while (!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK));
    }

    /* Enable FIRCDIV2 cho peripheral */
    SCG->FIRCDIV = SCG_FIRCDIV_FIRCDIV2(1);

    /* Enable clock for PORTC */
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Disable ADC0 clock before config */
    PCC->PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;

    /* choose FIRCDIV2 source clock for ADC0 */
    PCC->PCCn[PCC_ADC0_INDEX] = PCC_PCCn_PCS(3) | PCC_PCCn_CGC_MASK;

    /* Config PTC14 -> ADC0_SE12*/
    PORTC->PCR[14] = 0;

    /* Config ADC */
    ADC0->SC1[0] = ADC_SC1_ADCH_MASK;
    ADC0->CFG1   = ADC_CFG1_MODE(1)
                 | ADC_CFG1_ADIV(1)
                 | ADC_CFG1_ADICLK(0);
    ADC0->CFG2   = ADC_CFG2_SMPLTS(12);
    ADC0->SC2    = 0;
    ADC0->SC3    = 0;
}

/**
 * @brief Start ADC0 conversion
 *
 * @param channel ADC channel to read
 */
static void ADC0_Start(uint8_t channel)
{
    /* Start ADC0 conversion */
    ADC0->SC1[0] = ADC_SC1_ADCH(channel);
}

/**
 * @brief Read ADC0 conversion result
 *
 * @param channel ADC channel to read
 * @return uint16_t ADC conversion result
 */
uint16_t ADC0_Read(uint8_t channel)
{
    /* Variable to store ADC result */
    uint16_t retVal = 0;

    /* Start ADC0 conversion */
    ADC0_Start(channel);

    /* Wait for conversion to complete */
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));

    /* Read conversion result */
    retVal = (uint16_t)ADC0->R[0];

    return retVal;
}

/**
 * @brief Main function: Control LED color based on potentiometer value
 *
 * @return int
 */
int main(void)
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTD->PCR[LED_RED]   = PORT_PCR_MUX(1);
    PORTD->PCR[LED_GREEN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_BLUE]  = PORT_PCR_MUX(1);
    PTD->PDDR |= (1<<LED_RED) | (1<<LED_GREEN) | (1<<LED_BLUE);

    /* Turn off all LEDs (active low) */
    PTD->PSOR = (1<<LED_RED) | (1<<LED_GREEN) | (1<<LED_BLUE);

    /* Init ADC */
    ADC0_Init();

    while (1)
    {
        /* Read ADC value on channel 12 (PTC14 = potentiometer) */
        uint16_t adc_raw = ADC0_Read(12);

        /* Scale to 0–5000 mV */
        uint32_t mv = (uint32_t)adc_raw * SCALE_MV / ADC_MAX;

        /* Reset LED */
        PTD->PSOR = (1<<LED_RED) | (1<<LED_GREEN) | (1<<LED_BLUE);

        /* Compare according to the requirements */
        if (mv >= 3750 && mv <= 5000)
        {
            /* Turn on red LED */
            PTD->PCOR = (1<<LED_RED);
        } else if (mv >= 2500)
        {
            /* Turn on green LED */
            PTD->PCOR = (1<<LED_GREEN);
        } else if (mv >= 1250)
        {
            /* Turn on blue LED */
            PTD->PCOR = (1<<LED_BLUE);
        } else
        {
            /* Turn off all LEDs */
            PTD->PSOR = (1<<LED_RED) | (1<<LED_GREEN) | (1<<LED_BLUE);
        }
    }
}
