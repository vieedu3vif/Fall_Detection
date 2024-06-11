#include "gpio.h"

static GPIO_TypeDef* get_port_base(unsigned short PORT) {
    switch (PORT) {
        case PortA: return GPIOA;
        case PortB: return GPIOB;
        case PortC: return GPIOC;
        default: return (GPIO_TypeDef*)0; 
    }
}

static void enable_port_clock(unsigned short PORT) {
    switch (PORT) {
        case PortA: RCC->APB2ENR |= (1 << 2); break; // Enable GPIOA
        case PortB: RCC->APB2ENR |= (1 << 3); break; // Enable GPIOB
        case PortC: RCC->APB2ENR |= (1 << 4); break; // Enable GPIOC
    }
}

void gpio_init(unsigned short PORT, unsigned short PIN, unsigned int CNF, unsigned int MODE) {
    enable_port_clock(PORT);
    GPIO_TypeDef* port = get_port_base(PORT);

    if (port == (GPIO_TypeDef*)0) return;

    if (PIN < 8) {
        port->CRL &= ~(0xFU << (PIN * 4)); 
        port->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
    } else {
        port->CRH &= ~(0xFU << ((PIN - 8) * 4)); 
        port->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
    }

    if (CNF == IN_PUSHPULL && MODE == IN) {
        port->ODR |= (1 << PIN);
    }
}

void gpio_write(unsigned short PORT, unsigned short PIN, unsigned short value) {
    GPIO_TypeDef* port = get_port_base(PORT);

    if (port == (GPIO_TypeDef*)0) return;

    if (value == GPIO_PIN_SET) {
        port->BSRR = (1 << PIN);
    } else {
        port->BSRR = (1 << (PIN + 16));
    }
}

uint8_t gpio_read(unsigned short PORT, unsigned short PIN) {
    GPIO_TypeDef* port = get_port_base(PORT);

    if (port == (GPIO_TypeDef*)0) return GPIO_PIN_RESET;

    uint32_t pin_mask = (1 << PIN);
    return ((port->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
