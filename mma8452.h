#ifndef MMA8452_H
#define MMA8452_H

#include "stm32f10x.h"
#include "i2c.h"

// MMA8452Q I2C address
#define MMA8452Q_ADDRESS 0x1D

// MMA8452Q register addresses
#define MMA8452Q_WHO_AM_I 0x0D
#define MMA8452Q_CTRL_REG1 0x2A
#define MMA8452Q_X_OUT_MSB 0x01
#define MMA8452Q_Y_OUT_MSB 0x03
#define MMA8452Q_Z_OUT_MSB 0x05

// Function prototypes
uint8_t mma8452q_init(uint8_t i2c);
uint8_t mma8452q_read_register(uint8_t i2c, uint8_t reg);
void mma8452q_write_register(uint8_t i2c, uint8_t reg, uint8_t value);
void mma8452q_read_accel(uint8_t i2c, int16_t* x, int16_t* y, int16_t* z);
void mma8452q_set_active(uint8_t i2c, uint8_t active);
void mma8452q_set_data_rate(uint8_t i2c, uint8_t rate);

#endif
