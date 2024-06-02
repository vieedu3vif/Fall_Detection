#include "gpio.h"

void gpio_init(unsigned short PORT, unsigned short PIN, unsigned int CNF, unsigned int MODE) {
    // B?t clock cho c?ng GPIO tuong ?ng
    if (PORT == PortA) {
        RCC->APB2ENR |= (1 << 2); // enable GPIOA
    }
    else if (PORT == PortB) {
        RCC->APB2ENR |= (1 << 3); // enable GPIOB
    }
    else if (PORT == PortC) {
        RCC->APB2ENR |= (1 << 4); // enable GPIOC
    }

    // Thi?t l?p c?u hình chân GPIO
    if (PIN < 8) {
        // Chân 0-7 s? d?ng thanh ghi CRL
        if (PORT == PortA) {
            GPIOA->CRL &= ~(0xFU << (PIN * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOA->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
        else if (PORT == PortB) {
            GPIOB->CRL &= ~(0xFU << (PIN * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOB->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
        else if (PORT == PortC) {
            GPIOC->CRL &= ~(0xFU << (PIN * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOC->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
    } else {
        // Chân 8-15 s? d?ng thanh ghi CRH
        if (PORT == PortA) {
            GPIOA->CRH &= ~(0xFU << ((PIN - 8) * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOA->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
        else if (PORT == PortB) {
            GPIOB->CRH &= ~(0xFU << ((PIN - 8) * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOB->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
        else if (PORT == PortC) {
            GPIOC->CRH &= ~(0xFU << ((PIN - 8) * 4)); // clear 4 bit trong thanh ghi c?u hình
            GPIOC->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
    }

    // Thi?t l?p pull-up n?u CNF == IN_PUSHPULL (tuong ?ng v?i IN_PULLUP)
    if (CNF == IN_PUSHPULL && MODE == IN) {
        switch (PORT) {
            case PortA:
                GPIOA->ODR |= (1 << PIN);
                break;
            case PortB:
                GPIOB->ODR |= (1 << PIN);
                break;
            case PortC:
                GPIOC->ODR |= (1 << PIN);
                break;
            default:
                break;
        }
    }
}

void gpio_write(unsigned short PORT, unsigned short PIN, unsigned short value) {
    if (value == GPIO_PIN_SET) {
        switch (PORT) {
            case PortA:
                GPIOA->BSRR = (1 << PIN);
                break;
            case PortB:
                GPIOB->BSRR = (1 << PIN);
                break;
            case PortC:
                GPIOC->BSRR = (1 << PIN);
                break;
            default:
                break;
        }
    } else {
        switch (PORT) {
            case PortA:
                GPIOA->BSRR = (1 << (PIN + 16));
                break;
            case PortB:
                GPIOB->BSRR = (1 << (PIN + 16));
                break;
            case PortC:
                GPIOC->BSRR = (1 << (PIN + 16));
                break;
            default:
                break;
        }
    }
}

uint8_t gpio_read(unsigned short PORT, unsigned short PIN) {
    uint32_t pin_mask = (1 << PIN);  // Chuy?n d?i PIN thành giá tr? bit

    switch (PORT) {
        case PortA:
            return ((GPIOA->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        case PortB:
            return ((GPIOB->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        case PortC:
            return ((GPIOC->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        default:
            return GPIO_PIN_RESET; 
    }
}
