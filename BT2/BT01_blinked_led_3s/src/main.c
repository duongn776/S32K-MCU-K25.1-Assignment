/*
 * LED blinking on S32K: PD0, PD15, PD16 
 */

#include <stdint.h>

#define __vo volatile

/* define PCC */
#define PCC_BASE        0x40065000u
#define PCC_PORTD       (*(__vo uint32_t*)(PCC_BASE + 0x130))
#define CGC             30

/* define PortD */
#define PORTD_BASE      0x4004C000u
#define PORTD_PCR(n)    (*(__vo uint32_t*)(PORTD_BASE + (n * 0x4)))
#define MUX             8

/* define GPIOD */
#define GPIOD_BASE      0x400FF0C0u
#define GPIOD_PDOR      (*(__vo uint32_t*)(GPIOD_BASE + 0x0))
#define GPIOD_PSOR      (*(__vo uint32_t*)(GPIOD_BASE + 0x4))
#define GPIOD_PCOR      (*(__vo uint32_t*)(GPIOD_BASE + 0x8))
#define GPIOD_PTOR      (*(__vo uint32_t*)(GPIOD_BASE + 0xC))
#define GPIOD_PDDR      (*(__vo uint32_t*)(GPIOD_BASE + 0x14))

/* define PIN */
#define GPIO_PIN_0      0
#define GPIO_PIN_15     15
#define GPIO_PIN_16     16

/** Function Prototypes */
void PORTD_EnableClock(void);
void PORTD_Configure(uint8_t pin);
void GPIOD_SetPinOutput(uint8_t pin);
void delay(void);

/**
 * @brief Enable clock for PORTD
 * 
 */
void PORTD_EnableClock(void)
{
    PCC_PORTD |= (1 << CGC);
}

/**
 * @brief Configure pin as GPIO 
 * 
 * @param pin Pin number (0-31)
 */
void PORTD_Configure(uint8_t pin)
{
    PORTD_PCR(pin) &= ~0x700u;        // clear MUX bits [10:8]
    PORTD_PCR(pin) |=  (1 << MUX);    // set MUX = 001 (GPIO)
}

/**
 * @brief Set pin as output
 * 
 * @param pin Pin number (0-31)
 */
void GPIOD_SetPinOutput(uint8_t pin)
{
    GPIOD_PDDR |= (1 << pin);
}

/**
 * @brief Simple delay function
 * 
 */
void delay(void)
{
    for (volatile int i = 0; i < 10000000; i++); 
}

/**
 * @brief Main function: Turn on LEDs PD0, PD15, PD16 in sequence
 * 
 * @return int 
 */
int main(void)
{
    /* Enable clock for PORTD */
    PORTD_EnableClock();

    /* Configure pins PD0, PD15, PD16 */
    PORTD_Configure(GPIO_PIN_0);
    PORTD_Configure(GPIO_PIN_15);
    PORTD_Configure(GPIO_PIN_16);

    /* Set as output */
    GPIOD_SetPinOutput(GPIO_PIN_0);
    GPIOD_SetPinOutput(GPIO_PIN_15);
    GPIOD_SetPinOutput(GPIO_PIN_16);

    /* Init: all LEDs OFF */
    GPIOD_PSOR = (1 << GPIO_PIN_0) | (1 << GPIO_PIN_15) | (1 << GPIO_PIN_16);

    while (1) {
        /* Turn on LED PD0 */
        GPIOD_PCOR = (1 << GPIO_PIN_0);                 // ON
        GPIOD_PSOR = (1 << GPIO_PIN_15) | (1 << GPIO_PIN_16); // OFF others
        delay();

        /* Turn on LED PD15 */
        GPIOD_PCOR = (1 << GPIO_PIN_15);
        GPIOD_PSOR = (1 << GPIO_PIN_0) | (1 << GPIO_PIN_16);
        delay();

        /* Turn on LED PD16 */
        GPIOD_PCOR = (1 << GPIO_PIN_16);
        GPIOD_PSOR = (1 << GPIO_PIN_0) | (1 << GPIO_PIN_15);
        delay();
    }
}
