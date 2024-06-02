#include "stm32f10x.h" 
#include "systick.h"

void systick_init(void)
{
	SysTick->CTRL = 0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL = 0;
	SysTick->CTRL = 5; // clk source = 1, tickint = 0, enable = 1
}
static void Delaymicro(void)
{
	SysTick->LOAD = SystemCoreClock / 1000000;
	SysTick->VAL = 0;
	while((SysTick->CTRL & 0x00010000) == 0);
}

void delay_us(unsigned long t)
{
	for(;t>0;t--)
		{
			Delaymicro();
		}
}


static void DelayMillis(void)
{
	SysTick->LOAD = SystemCoreClock / 1000;
	SysTick->VAL = 0;
	while((SysTick->CTRL & 0x00010000) == 0);
}

void delay_ms(unsigned long t)
{
	for(;t>0;t--)
		{
			DelayMillis();
		}
}
