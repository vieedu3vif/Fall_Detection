#include "main.h"
#include "stm32f10x.h"

	volatile int STATE = 0;
	volatile int FALL = 1;
	volatile int LEDG = 0;
	volatile int LEDR = 0;
	
	void SystemClock_Config(void) {
   
    RCC->CR |= RCC_CR_HSEON;

   
    while (!(RCC->CR & RCC_CR_HSERDY));

    
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;  // 2 wait states for 72 MHz

    
    RCC->CFGR |= RCC_CFGR_PLLSRC;       // HSE oscillator clock selected as PLL input clock
    RCC->CFGR |= RCC_CFGR_PLLMULL9;     // PLL input clock x 9

   
    RCC->CR |= RCC_CR_PLLON;

 
    while (!(RCC->CR & RCC_CR_PLLRDY));

 
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;    // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // PCLK1 = HCLK/2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;   // PCLK2 = HCLK

   
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

   
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    
    SystemCoreClockUpdate();
}
int main (void) {
	SystemClock_Config();
	timer_init();
	
  i2c_init(I2C_1, I2C_FM);
	i2c_init(I2C_1, I2C_FM);

   
    lcd_i2c_init(I2C_1);

   
  button_init(0);
	button_init(sw2);
	led_init(5);
	led_init(2);
	  intr_init();
	 lcd_i2c_msg(I2C_1, 1, 0, "Vietduc03 - A+");
//	uart_init(UART3, BR_115200);			
 while(1) {

	// delay_ms(500);
	 //	lcd_i2c_cmd(I2C_1, 0x01); 
	 	// delay_ms(500); 
	 if (STATE == 1) {
           
        //    led_blinked(5);
		        if( LEDG == 1) {
							led_on(5);
							}
						else {
							led_off(5);
							} 
		        if( FALL == 1 ){
							if( LEDR == 1) {
								led_on(2);
							}
							else {
								led_off(2);
							}
						}
						else {
							led_off(2);
							}
        } else {
						led_off(2);
            led_off(5);
        }
	 
    }
}

void EXTI0_IRQHandler(void) {
 
    if (EXTI->PR & (1 << 0 )) {
        EXTI->PR |= (1 << 0 );
			STATE = !STATE;
   } 
} 

void EXTI1_IRQHandler(void) {
	if (EXTI->PR & (1 << 1)) {   
      EXTI->PR |= (1 << 1);
		  FALL = !FALL;
		}
}
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) // If update interrupt flag is set
    {
        TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
        LEDG = !LEDG;
    }
} 

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) { 
        TIM3->SR &= ~TIM_SR_UIF; 
        LEDR = !LEDR; 
    }
}
void EXTI3_IRQHandler(void) {
	if (EXTI->PR & (1 << 3)) {   
      EXTI->PR |= (1 << 3);
		  FALL = !FALL;
		}
}
