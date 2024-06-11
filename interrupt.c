#include "interrupt.h"



void intr_init(){
	
	RCC->APB2ENR |= (1 << 0);
	__disable_irq();
	
	
	  AFIO->EXTICR[0]  &= ~(0xFU << 0);
    AFIO->EXTICR[0] |= (0x0 << 0);   
	
  	EXTI->IMR &= ~(unsigned int)(1 << 0);
  	EXTI->IMR |= (1 << 0 );
	  EXTI->FTSR |= (1 << 0 );
	  EXTI->RTSR &= ~(unsigned int)(1 << 0 );

    
		AFIO->EXTICR[0] &= ~(0xFU << 12); 
		AFIO->EXTICR[0] |= (0x0 << 12);   
	
    EXTI->IMR &= ~(unsigned int)(1 << 3); 
    EXTI->IMR |= (1 << 3); 
    EXTI->FTSR |= (1 << 3); 
    EXTI->RTSR &= ~(unsigned int)(1 << 3); 
		
		NVIC_SetPriority(EXTI0_IRQn, 0);
	  NVIC_EnableIRQ(EXTI0_IRQn);
		
    NVIC_SetPriority(EXTI3_IRQn, 1); 
    NVIC_EnableIRQ(EXTI3_IRQn); 
		
	  NVIC_SetPriority(TIM2_IRQn, 2); 
    NVIC_EnableIRQ(TIM2_IRQn);
		
		NVIC_SetPriority(TIM3_IRQn, 2); 
    NVIC_EnableIRQ(TIM3_IRQn);

	__enable_irq();	
	
}

