#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f10x.h"
#include "gpio.h"


#define sw1 0
#define sw2 3
void button_init(unsigned short button);


#endif // BUTTON_H
