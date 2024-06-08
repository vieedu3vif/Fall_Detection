#include "main.h"
#include "stm32f10x.h"

	volatile int STATE = 0;
	volatile int FALL = 0;
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
	i2c_init(I2C_2, I2C_FM);
	lcd_i2c_init(I2C_1);
  MPU6050_Init(I2C_2);
	MPU6050 mpu;
   
  button_init(sw1);
	button_init(sw2);
	led_init(5);
	led_init(2);
	intr_init();
//	 	 lcd_i2c_msg(I2C_1, 1, 0, "Vietduc03 - B+");
//	uart_init(UART3, BR_115200);	
char buffer[50];	
 while(1) {
/*	 MPU6050_Read_Accel(I2C_2, &mpu);
	 snprintf(buffer, sizeof(buffer), "X: %.2f, Y: %.2f, Z: %.2f", mpu.Ax, mpu.Ay, mpu.Az);
	 lcd_i2c_msg(I2C_1, 1,0,  buffer);
	 delay_ms(500);
  	lcd_i2c_cmd(I2C_1, 0x01); 
	 	// delay_ms(500); */
	 
     if (STATE == 1) {
			 fallDetect(I2C_2, &mpu);
		        if( LEDG == 1) {
							led_on(5);
							}
						else {
							led_off(5);
							} 
		        if( FALL == 1 ){
							lcd_i2c_msg(I2C_1, 1,0,  "YOU FALL");
							
							if( LEDR == 1) {
								led_on(2);
							}
							else {
								led_off(2);
							}
						}
						
						
						else {
							lcd_i2c_msg(I2C_1, 1,0,  "NORMAL!!");
						/*	 snprintf(buffer, sizeof(buffer), "X: %.2f, Y: %.2f, Z: %.2f", mpu.Gx, mpu.Gy, mpu.Gz);
            	 lcd_i2c_msg(I2C_1, 1,0,  buffer); */
							led_off(2);
							}
        } else {
					  FALL = 0;
					  lcd_i2c_cmd(I2C_1, 0x01); 
					  delay_ms(100);
						led_off(2);
            led_off(5);
        } 
	 
/*	 	snprintf(buffer, sizeof(buffer), "%d, %d", STATE, FALL);
	 lcd_i2c_msg(I2C_1, 1,0,  buffer);
	 delay_ms(100);
  	lcd_i2c_cmd(I2C_1, 0x01); 
		delay_ms(100);   */
    }  
}

void EXTI0_IRQHandler(void) {
 
    if (EXTI->PR & (1 << 0 )) {
        EXTI->PR |= (1 << 0 );
			 delay_ms(20); 
        if (GPIOA->IDR & (1 << 0)) {
            STATE = !STATE;
        }
   } 
} 

/*
void EXTI1_IRQHandler(void) {
	if (EXTI->PR & (1 << 1)) {   
      EXTI->PR |= (1 << 1);
		  FALL = !FALL;
		}
} */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) 
    {
        TIM2->SR &= ~TIM_SR_UIF; 
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
		  			 delay_ms(20); 
        if (GPIOA->IDR & (1 << 3)) {
            FALL = 0;
        }
		}
}

void fallDetect(uint8_t i2c, MPU6050 *mpu) {
    double acc[6]; 

    MPU6050_Read_Gyro(i2c, mpu);
    acc[0] = mpu->Gx;
    acc[1] = mpu->Gy;
    acc[2] = mpu->Gz;
    
    delay_ms(100);
    MPU6050_Read_Gyro(i2c, mpu);
    acc[3] = mpu->Gx;
    acc[4] = mpu->Gy;
    acc[5] = mpu->Gz;
   
    if((fabs(acc[3] + acc[4] + acc[5]) - fabs(acc[0] - acc[1] - acc[2])) > 80) {
		FALL = 1;
    }
} 
