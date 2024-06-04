#include "i2c.h"

I2C_Status i2c_init(uint8_t i2c, unsigned short speed_mode) {

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    
    if (i2c == I2C_1) {
			  gpio_init(PortB, 6, OUT_AF_OD, OUT50);
        gpio_init(PortB, 7, OUT_AF_OD, OUT50);
			
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        I2C1->CR2 = 0x8;  
        I2C1->CCR = speed_mode; 
        I2C1->CR1 |= I2C_CR1_PE; 
    } else if (i2c == I2C_2) {
			  gpio_init(PortB, 10, OUT_AF_OD, OUT50);
        gpio_init(PortB, 11, OUT_AF_OD, OUT50);
			
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        I2C2->CR2 = 0x8;  
        I2C2->CCR = speed_mode;  
        I2C2->CR1 |= I2C_CR1_PE;  
    }

    return I2C_Success;
}

I2C_Status i2c_start(uint8_t i2c) {
    if (i2c == I2C_1) {
        I2C1->CR1 |= I2C_CR1_START;
        while (!(I2C1->SR1 & I2C_SR1_SB)); 
    } else if (i2c == I2C_2) {
        I2C2->CR1 |= I2C_CR1_START;
        while (!(I2C2->SR1 & I2C_SR1_SB)); 
    }

    return I2C_Success;
}

I2C_Status i2c_add(uint8_t i2c, char address, char RW) {
    if (i2c == I2C_1) {
        I2C1->DR = (address | RW);
        while (!(I2C1->SR1 & I2C_SR1_ADDR));  
        (void) I2C1->SR2;  
    } else if (i2c == I2C_2) {
        I2C2->DR = (address | RW);
        while (!(I2C2->SR1 & I2C_SR1_ADDR));  
        (void) I2C2->SR2; 
    }

    return I2C_Success;
}

I2C_Status i2c_data(uint8_t i2c, char data) {
    if (i2c == I2C_1) {
        I2C1->DR = data;
        while (!(I2C1->SR1 & I2C_SR1_TXE)); 
    } else if (i2c == I2C_2) {
        I2C2->DR = data;
        while (!(I2C2->SR1 & I2C_SR1_TXE));
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
uint8_t i2c_read(uint8_t i2c, uint8_t ack) {
    uint8_t data;
    if (i2c == I2C_1) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        data = I2C1->DR;
        if (ack) {
            I2C1->CR1 |= I2C_CR1_ACK;
        } else {
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
    } else if (i2c == I2C_2) {
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        data = I2C2->DR;
        if (ack) {
            I2C2->CR1 |= I2C_CR1_ACK;
        } else {
            I2C2->CR1 &= ~I2C_CR1_ACK;
        }
    }
    return data;
}

