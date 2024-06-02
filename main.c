#include "main.h"
#include "stm32f10x.h"


	volatile int STATE = 0;
	volatile int FALL = 0;
	volatile int LEDG = 0;
	volatile int LEDR = 0;
	
int main (void) {
	
	timer_init();
	
   i2c_init(I2C_1, I2C_FM);
	i2c_init(I2C_2, I2C_FM);

    // Kh?i t?o màn hình LCD
    lcd_i2c_init(I2C_1);

    // Hi?n th? thông di?p trên màn hình LCD
  button_init(0);
	led_init(5);
	led_init(2);
	  intr_init();

	uart_init(UART3, BR_115200);
 while(1) {
	/* lcd_i2c_msg(I2C_1, 1, 0, "Vietduc03??");
	 delay_ms(500);
	 	lcd_i2c_cmd(I2C_1, 0x01); // Clear Display // rs = 1	
	 	 delay_ms(500);
	 lcd_i2c_msg(I2C_1, 1, 0, "University of ET");
	 delay_ms(500); */
			//	led_blinked(13);
	  //  led_blinked(5);
        // G?i d? li?u qua UART
     //   snprintf(buffer, sizeof(buffer), "X: %d, Y: %d, Z: %d\r\n", x, y, z);
    //    uart_send_msg(UART3, buffer);
	 
	 		gpio_init(PortA, 1, OUT_GP_PP, OUT2);
		  gpio_write(PortA, 1, GPIO_PIN_SET);
	    GPIOA->BRR = (1 << 1); 
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

