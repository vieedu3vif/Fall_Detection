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
