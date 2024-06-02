#include "timer.h"
#include "stm32f10x.h"

void timer_init(void)
{
   
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    
    TIM4->PSC = (SystemCoreClock / 1000000) - 1;
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 = TIM_CR1_CEN;
	
	  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	  TIM2->PSC = ((uint16_t)(SystemCoreClock / 1000)) - 1;
    TIM2->ARR = 5000-1;  
    TIM2->DIER |= TIM_DIER_UIE;  
    TIM2->CR1 |= TIM_CR1_CEN;  
	   
	  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	  TIM3->PSC = ((uint16_t)(SystemCoreClock / 1000)) - 1;  
    TIM3->ARR = (2500-1);
    TIM3->DIER |= TIM_DIER_UIE;  
    TIM3->CR1 |= TIM_CR1_CEN; 
}


void delay_us(unsigned long t)
{
    TIM4->CNT = 0;
    while (TIM4->CNT < t);
}

void delay_ms(unsigned long t)
{
    for (unsigned long i = 0; i < t; i++) {
        delay_us(1000);
    }
}
