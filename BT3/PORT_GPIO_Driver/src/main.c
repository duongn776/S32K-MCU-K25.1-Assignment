#include "Driver_PORT.h"
#include "Driver_GPIO.h"

void delay(volatile uint32_t count)
{
    while (count--) {
        __asm("NOP");   // tránh bị tối ưu hóa
    }
}

int main(void)
{
    // Enable clock cho PORTC, PORTD
    PORT_PeriClockControl(PORTC, ENABLE);
    PORT_PeriClockControl(PORTD, ENABLE);

    // Config LED Red (PD0)
    PORT_Config_t led_red_cfg = {
        .portBase  = PORTD,
        .pin       = 0,
        .mux       = PORT_MUX_GPIO,
        .pull      = PORT_NOPULL,
        .interrupt = PORT_INT_DISABLED
    };
    PORT_Init(&led_red_cfg);
    GPIO_SetPinDirection(IP_PTD, 0, 1);
    GPIO_SetPin(IP_PTD, 0);   // OFF (active low)

    // Config LED Green (PD15)
    PORT_Config_t led_green_cfg = {
        .portBase  = PORTD,
        .pin       = 15,
        .mux       = PORT_MUX_GPIO,
        .pull      = PORT_NOPULL,
        .interrupt = PORT_INT_DISABLED
    };
    PORT_Init(&led_green_cfg);
    GPIO_SetPinDirection(IP_PTD, 15, 1);
    GPIO_SetPin(IP_PTD, 15);  // OFF (active low)

    // Config Button SW2 (PTC12)
    PORT_Config_t btn1_cfg = {
        .portBase  = PORTC,
        .pin       = 12,
        .mux       = PORT_MUX_GPIO,
        .pull      = PORT_PULLUP,
        .interrupt = PORT_INT_DISABLED
    };
    PORT_Init(&btn1_cfg);
    GPIO_SetPinDirection(IP_PTC, 12, 0);  // Input

    // Config Button SW3 (PTC13)
    PORT_Config_t btn2_cfg = {
        .portBase  = PORTC,
        .pin       = 13,
        .mux       = PORT_MUX_GPIO,
        .pull      = PORT_PULLUP,
        .interrupt = PORT_INT_DISABLED
    };
    PORT_Init(&btn2_cfg);
    GPIO_SetPinDirection(IP_PTC, 13, 0);  // Input

    uint8_t last_btn1 = 1, last_btn2 = 1;

    while (1) {
        // Button 1 → Toggle Red LED (PD0)
        uint8_t btn1 = GPIO_ReadPin(IP_PTC, 12);
        if (btn1 == 0 && last_btn1 == 1) {
            delay(100000); // debounce
            if (GPIO_ReadPin(IP_PTC, 12) == 0) {
                GPIO_TogglePin(IP_PTD, 0);  // Toggle LED đỏ
            }
        }
        last_btn1 = btn1;

        // Button 2 → Toggle Green LED (PD15)
        uint8_t btn2 = GPIO_ReadPin(IP_PTC, 13);
        if (btn2 == 0 && last_btn2 == 1) {
            delay(100000); // debounce
            if (GPIO_ReadPin(IP_PTC, 13) == 0) {
                GPIO_TogglePin(IP_PTD, 15); // Toggle LED xanh
            }
        }
        last_btn2 = btn2;
    }
}
