#include "button.h"

void button_init(unsigned short button)
{
	if( button == sw1 ) {
		gpio_init(PortA, 0, IN_PUSHPULL, IN);
		
		}
	else if (button == sw2) {
		gpio_init(PortA, 3, IN_PUSHPULL, IN);
		}
}

uint8_t button_read(unsigned short button)
{
    if(button == sw1) {
        return gpio_read(PortA, 0); 
    } else if(button == sw2) {
        
        return gpio_read(PortA, 1); 
    } else {
       
        return GPIO_PIN_ERR;
    }
}
