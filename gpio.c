#include "gpio.h"

void gpio_init(unsigned short PORT, unsigned short PIN, unsigned int CNF, unsigned int MODE) {
 
    if (PORT == PortA) {
        RCC->APB2ENR |= (1 << 2); // enable GPIOA
    }
    else if (PORT == PortB) {
        RCC->APB2ENR |= (1 << 3); // enable GPIOB
    }
    else if (PORT == PortC) {
        RCC->APB2ENR |= (1 << 4); // enable GPIOC
    }

  
    if (PIN < 8) {
        
        if (PORT == PortA) {
            GPIOA->CRL &= ~(0xFU << (PIN * 4)); 
            GPIOA->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
        else if (PORT == PortB) {
            GPIOB->CRL &= ~(0xFU << (PIN * 4)); 
            GPIOB->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
        else if (PORT == PortC) {
            GPIOC->CRL &= ~(0xFU << (PIN * 4));
            GPIOC->CRL |= (MODE << (PIN * 4)) | (CNF << (PIN * 4 + 2));
        }
    } else {
       
        if (PORT == PortA) {
            GPIOA->CRH &= ~(0xFU << ((PIN - 8) * 4)); 
            GPIOA->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
        else if (PORT == PortB) {
            GPIOB->CRH &= ~(0xFU << ((PIN - 8) * 4)); 
            GPIOB->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
        else if (PORT == PortC) {
            GPIOC->CRH &= ~(0xFU << ((PIN - 8) * 4));
            GPIOC->CRH |= (MODE << ((PIN - 8) * 4)) | (CNF << ((PIN - 8) * 4 + 2));
        }
    }

   
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
    uint32_t pin_mask = (1 << PIN);  

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
