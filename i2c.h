#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"  // �ua gpio.h l�n tr�n systick.h v� n� du?c s? d?ng trong i2c.h

// �?nh nghia h?ng s? speed mode
#define I2C_FM 0x2D
#define I2C_SM 0xB4

// �?nh nghia h?ng s? ACK v� NACK
#define ACK 0
#define NACK 1

#define I2C_1 0
#define I2C_2 1
// �?nh nghia ki?u d? li?u v� h?ng s? enum Status
typedef enum {I2C_Error = 0, I2C_Success = !I2C_Error} I2C_Status;

// �?nh nghia c�c h�m API cho thu vi?n I2C
I2C_Status i2c_init(uint8_t i2c, unsigned short speed_mode);
I2C_Status i2c_add(uint8_t i2c, char address, char RW);
I2C_Status i2c_write(uint8_t i2c, char address, char data[]);
I2C_Status i2c_data(uint8_t i2c, char data);
I2C_Status i2c_start(uint8_t i2c);
I2C_Status i2c_stop(uint8_t i2c);

#endif