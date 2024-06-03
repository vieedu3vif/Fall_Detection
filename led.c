
#include "led.h"


#define LEDGREEN_PIN     (1 << 2) // Pin A2
#define LEDRED_PIN   (1 << 5)  // Pin B5


void led_init(unsigned int led)
{
    if (led == 2) {
        gpio_init(PortA, 2, OUT_GP_PP, OUT2); 
    } else {
        gpio_init(PortB, 5, OUT_GP_PP, OUT2);
    }
}

void led_on(unsigned int led)
{
    if (led == 2) {
			
        gpio_write(PortA, 2, GPIO_PIN_SET); 
    } else {
        gpio_write(PortB, 5, GPIO_PIN_SET); 
    }
}

void led_off(unsigned int led)
{
    if (led == 2) {
        GPIOA->BRR = LEDGREEN_PIN; 
    } else {
        GPIOB->BRR = LEDRED_PIN; 
    }
}
void led_blinked (unsigned int led) {
    if(led == 2) {
			led_on(led);
			delay_ms(1000);
			led_off(led);
			delay_ms(1000);
	  }
		else {
			led_on(led);
			delay_ms(1000);
			led_off(led);
			delay_ms(1000);
			}
}
