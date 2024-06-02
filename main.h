#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "led.h"
#include "i2c.h"
#include "lcd.h"
#include "uart.h"
#include "interrupt.h"
#include "button.h"
#include "gpio.h"


extern volatile int LEDG;
extern volatile int STATE;
extern volatile int LEDR;
extern volatile int FALL;


int main(void);
