#include "stm32f10x.h"
#include "timer.h"
#include "gpio.h"
#include <stdbool.h>


void led_init(unsigned int led);
void led_on(unsigned int led);
void led_off(unsigned int led);
void led_blinked (unsigned int led);
