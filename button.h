#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f10x.h"
#include "gpio.h"


#define sw1 0
#define sw2 1
void button_init(unsigned short button);
uint8_t button_read(unsigned short button);


#endif // BUTTON_H
