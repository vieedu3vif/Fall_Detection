#include "interrupt.h"



void intr_init(){
	
	RCC->APB2ENR |= (1 << 0);
	__disable_irq();
	AFIO->EXTICR[0]  &= ~(0xFU << 0);

	
	EXTI->IMR &= ~(unsigned int)(1 << 0);
	EXTI->IMR |= (1 << 0 );
	EXTI->FTSR |= (1 << 0 );
	EXTI->RTSR &= ~(unsigned int)(1 << 0 );



	NVIC_SetPriority(EXTI0_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_IRQn);
	 
    AFIO->EXTICR[0] &= ~(0xFU << 4); 
    
    EXTI->IMR &= ~(unsigned int)(1 << 1); 
    EXTI->IMR |= (1 << 1); 
    EXTI->FTSR |= (1 << 1); 
    EXTI->RTSR &= ~(unsigned int)(1 << 1); 
    
    NVIC_SetPriority(EXTI1_IRQn, 1); 
    NVIC_EnableIRQ(EXTI1_IRQn); 
	  NVIC_SetPriority(TIM2_IRQn, 2); 
    NVIC_EnableIRQ(TIM2_IRQn);
		NVIC_SetPriority(TIM3_IRQn, 3); 
    NVIC_EnableIRQ(TIM3_IRQn);
	

	__enable_irq();	
	
}

