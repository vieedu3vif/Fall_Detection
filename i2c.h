#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"  


#define I2C_FM 0x2D
#define I2C_SM 0xB4


#define ACK 0
#define NACK 1

#define I2C_1 0
#define I2C_2 1

typedef enum {I2C_Error = 0, I2C_Success = !I2C_Error} I2C_Status;

I2C_Status i2c_init(uint8_t i2c, unsigned short speed_mode);
I2C_Status i2c_add(uint8_t i2c, char address, char RW);
I2C_Status i2c_write(uint8_t i2c, char address, char data[]);
I2C_Status i2c_data(uint8_t i2c, char data);
I2C_Status i2c_start(uint8_t i2c);
I2C_Status i2c_stop(uint8_t i2c);
uint8_t i2c_read(uint8_t i2c, uint8_t ack);

#endif
