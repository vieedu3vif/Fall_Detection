#include "mma8452.h"

// Initialize the MMA8452Q
uint8_t mma8452q_init(uint8_t i2c) {
    uint8_t who_am_i = mma8452q_read_register(i2c, MMA8452Q_WHO_AM_I);
    if (who_am_i != 0x2A) {  // MMA8452Q WHO_AM_I register should return 0x2A
        return 0;  // Initialization failed
    }
    mma8452q_write_register(i2c, MMA8452Q_CTRL_REG1, 0x00);  // Set standby mode
    mma8452q_set_data_rate(i2c, 0x00);  // Set data rate to 800Hz
    mma8452q_write_register(i2c, MMA8452Q_CTRL_REG1, 0x01);  // Set active mode
    return 1;  // Initialization succeeded
}

// Read a register from the MMA8452Q
uint8_t mma8452q_read_register(uint8_t i2c, uint8_t reg) {
    uint8_t value = 0;
    i2c_start(i2c);
    i2c_add(i2c, MMA8452Q_ADDRESS, 0);
    i2c_data(i2c, reg);
    i2c_start(i2c);
    i2c_add(i2c, MMA8452Q_ADDRESS, 1);
    value = i2c_data(i2c, NACK);
    i2c_stop(i2c);
    return value;
}

// Write a register on the MMA8452Q
void mma8452q_write_register(uint8_t i2c, uint8_t reg, uint8_t value) {
    i2c_start(i2c);
    i2c_add(i2c, MMA8452Q_ADDRESS, 0);
    i2c_data(i2c, reg);
    i2c_data(i2c, value);
    i2c_stop(i2c);
}

// Read accelerometer data
void mma8452q_read_accel(uint8_t i2c, int16_t* x, int16_t* y, int16_t* z) {
    uint8_t x_msb = mma8452q_read_register(i2c, MMA8452Q_X_OUT_MSB);
    uint8_t x_lsb = mma8452q_read_register(i2c, MMA8452Q_X_OUT_MSB + 1);
    uint8_t y_msb = mma8452q_read_register(i2c, MMA8452Q_Y_OUT_MSB);
    uint8_t y_lsb = mma8452q_read_register(i2c, MMA8452Q_Y_OUT_MSB + 1);
    uint8_t z_msb = mma8452q_read_register(i2c, MMA8452Q_Z_OUT_MSB);
    uint8_t z_lsb = mma8452q_read_register(i2c, MMA8452Q_Z_OUT_MSB + 1);

    *x = (int16_t)((x_msb << 8) | x_lsb) >> 4;
    *y = (int16_t)((y_msb << 8) | y_lsb) >> 4;
    *z = (int16_t)((z_msb << 8) | z_lsb) >> 4;
}

// Set the active state of the MMA8452Q
void mma8452q_set_active(uint8_t i2c, uint8_t active) {
    uint8_t ctrl_reg1 = mma8452q_read_register(i2c, MMA8452Q_CTRL_REG1);
    if (active) {
        ctrl_reg1 |= 0x01;  // Set the active bit
    } else {
        ctrl_reg1 &= ~0x01;  // Clear the active bit
    }
    mma8452q_write_register(i2c, MMA8452Q_CTRL_REG1, ctrl_reg1);
}

// Set the data rate of the MMA8452Q
void mma8452q_set_data_rate(uint8_t i2c, uint8_t rate) {
    uint8_t ctrl_reg1 = mma8452q_read_register(i2c, MMA8452Q_CTRL_REG1);
    ctrl_reg1 &= ~0x38;  // Clear the data rate bits
    ctrl_reg1 |= (rate << 3);  // Set the new data rate
    mma8452q_write_register(i2c, MMA8452Q_CTRL_REG1, ctrl_reg1);
}
