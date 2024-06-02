#include "i2c.h"

I2C_Status i2c_init(uint8_t i2c, unsigned short speed_mode) {
    // B?t clock cho GPIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // C?u hình chân SCL và SDA
    gpio_init(PortB, 6, OUT_AF_OD, OUT50);
    gpio_init(PortB, 7, OUT_AF_OD, OUT50);
	   gpio_init(PortB, 10, OUT_AF_OD, OUT50);
    gpio_init(PortB, 11, OUT_AF_OD, OUT50);

    // B?t clock cho I2C
    if (i2c == I2C_1) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        I2C1->CR2 = 0x8;  // Cài d?t t?n s? 8MHz
        I2C1->CCR = speed_mode;  // Cài d?t t?c d? truy?n
        I2C1->CR1 |= I2C_CR1_PE;  // B?t I2C1
    } else if (i2c == I2C_2) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        I2C2->CR2 = 0x8;  // Cài d?t t?n s? 8MHz
        I2C2->CCR = speed_mode;  // Cài d?t t?c d? truy?n
        I2C2->CR1 |= I2C_CR1_PE;  // B?t I2C2
    }

    return I2C_Success;
}

I2C_Status i2c_start(uint8_t i2c) {
    if (i2c == I2C_1) {
        I2C1->CR1 |= I2C_CR1_START;
        while (!(I2C1->SR1 & I2C_SR1_SB));  // Ğ?i t?i khi tr?ng thái start du?c xác nh?n
    } else if (i2c == I2C_2) {
        I2C2->CR1 |= I2C_CR1_START;
        while (!(I2C2->SR1 & I2C_SR1_SB));  // Ğ?i t?i khi tr?ng thái start du?c xác nh?n
    }

    return I2C_Success;
}

I2C_Status i2c_add(uint8_t i2c, char address, char RW) {
    if (i2c == I2C_1) {
        I2C1->DR = (address | RW);
        while (!(I2C1->SR1 & I2C_SR1_ADDR));  // Ğ?i t?i khi tr?ng thái d?a ch? du?c xác nh?n
        (void) I2C1->SR2;  // Ğ?c SR2 d? clear di tr?ng thái d?a ch?
    } else if (i2c == I2C_2) {
        I2C2->DR = (address | RW);
        while (!(I2C2->SR1 & I2C_SR1_ADDR));  // Ğ?i t?i khi tr?ng thái d?a ch? du?c xác nh?n
        (void) I2C2->SR2;  // Ğ?c SR2 d? clear di tr?ng thái d?a ch?
    }

    return I2C_Success;
}

I2C_Status i2c_data(uint8_t i2c, char data) {
    if (i2c == I2C_1) {
        I2C1->DR = data;
        while (!(I2C1->SR1 & I2C_SR1_TXE));  // Ğ?i t?i khi thanh ghi truy?n r?ng
    } else if (i2c == I2C_2) {
        I2C2->DR = data;
        while (!(I2C2->SR1 & I2C_SR1_TXE));  // Ğ?i t?i khi thanh ghi truy?n r?ng
    }

    return I2C_Success;
}

I2C_Status i2c_stop(uint8_t i2c) {
    if (i2c == I2C_1) {
        I2C1->CR1 |= I2C_CR1_STOP;
    } else if (i2c == I2C_2) {
        I2C2->CR1 |= I2C_CR1_STOP;
    }

    return I2C_Success;
}

I2C_Status i2c_write(uint8_t i2c, char address, char data[]) {
    uint32_t i = 0;
    i2c_start(i2c);
    i2c_add(i2c, address, 0);
    while (data[i] != '\0') {
        i2c_data(i2c, data[i]);
        i++;
    }
    i2c_stop(i2c);

    return I2C_Success;
}
