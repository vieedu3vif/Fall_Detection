#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "led.h"
#include "i2c.h"
#include "lcd.h"
#include "interrupt.h"
#include "button.h"
#include "gpio.h"
#include "mpu6050.h"

extern volatile int LEDG;
extern volatile int STATE;
extern volatile int LEDR;
extern volatile int FALL;
extern volatile bool in_free_fall;
void SystemClock_Config(void);
void fallDetect(uint8_t i2c, MPU6050 *mpu);
int main(void);
