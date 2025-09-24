#include "S32K144.h"

#define LED_PIN   0   // PD0
#define BTN_PIN   12  // PC12

#define PCC		IP_PCC
/* Alias for PORTx */
#define PORTA   IP_PORTA
#define PORTB   IP_PORTB
#define PORTC   IP_PORTC
#define PORTD   IP_PORTD
#define PORTE   IP_PORTE
#define PTD		IP_PTD
#define PTC		IP_PTC
/* Delay đơn giản để debounce */
static void delay_ms(volatile uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000; i++) {
        __asm("nop");
    }
}

int main(void)
{
    /* Bật clock cho PORTC và PORTD (PCC = Peripheral Clock Control) */
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Cấu hình chân PD0 là GPIO */
    PORTD->PCR[LED_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= (1 << LED_PIN);     // output
    PTD->PSOR = (1 << LED_PIN);      // LED off ban đầu (tùy mạch LED active high hay low)

    /* Cấu hình chân PC12 là GPIO input + pull-up */
    PORTC->PCR[BTN_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTC->PDDR &= ~(1 << BTN_PIN);    // input

    uint8_t led_state = 0;
    uint8_t last_btn = (PTC->PDIR & (1 << BTN_PIN)) ? 1 : 0;

    while (1)
    {
        uint8_t cur_btn = (PTC->PDIR & (1 << BTN_PIN)) ? 1 : 0;

        if (cur_btn != last_btn) {
            delay_ms(20);  // debounce
            cur_btn = (PTC->PDIR & (1 << BTN_PIN)) ? 1 : 0;
            if (cur_btn != last_btn) {
                last_btn = cur_btn;
                if (cur_btn == 0) {  // nhấn nút (active low)
                    led_state ^= 1;  // toggle
                    if (led_state) {
                        PTD->PCOR = (1 << LED_PIN); // LED ON
                    } else {
                        PTD->PSOR = (1 << LED_PIN); // LED OFF
                    }
                }
            }
        }
    }
}
