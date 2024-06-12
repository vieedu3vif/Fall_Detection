#include "main.h"
#include "stm32f10x.h"

	volatile int STATE = 0;
	volatile int FALL = 0;
	volatile int LEDG = 0;
	volatile int LEDR = 0;
	
void SystemClock_Config(void) {
   
    RCC->CR |= (1 << 0); // Set HSEON bit
    while (!(RCC->CR & (1 << 17))); // Wait until HSERDY bit is set

    FLASH->ACR |= (1 << 4); // Enable Flash prefetch buffer
    FLASH->ACR &= ~(unsigned int)(0x7 << 0); // Clear LATENCY bits
    FLASH->ACR |= (2 << 0); // Set LATENCY bits for 72 MHz
 
    RCC->CFGR |= (1 << 16); // HSE oscillator clock selected as PLL input clock
    RCC->CFGR |= (9 << 18); // PLL input clock x 9
 
    RCC->CR |= (1 << 24); // Set PLLON bit
    while (!(RCC->CR & (1 << 25))); // Wait until PLLRDY bit is set

    RCC->CFGR |= (0 << 4); // HCLK = SYSCLK
    RCC->CFGR |= (2 << 8); // PCLK1 = HCLK/2
    RCC->CFGR |= (0 << 11); // PCLK2 = HCLK
   
    RCC->CFGR &= ~(unsigned int)(0x3 << 0); // Clear SW bits
    RCC->CFGR |= (2 << 0); // Set SW bits to PLL
    while ((RCC->CFGR & (0x3 << 2)) != (2 << 2)); // Wait until PLL is used as system clock

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
 while(1) {
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
							led_off(2);
							}
        } else {
					  FALL = 0;
					  lcd_i2c_cmd(I2C_1, 0x01); 
					  delay_ms(100);
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
            FALL = 0;
		
		
		}
}

void fallDetect(uint8_t i2c, MPU6050 *mpu) {
    double acc[6]; 

    MPU6050_Read_Accel(i2c, mpu);
	  MPU6050_Read_Gyro(i2c, mpu);
	
    acc[0] = mpu->Ax;
    acc[1] = mpu->Ay;
    acc[2] = mpu->Az;
    
    acc[3] = mpu->Gx;
    acc[4] = mpu->Gy;
    acc[5] = mpu->Gz;
   
    if(sqrt(pow(acc[0], 2)+ pow(acc[1], 2)+pow(acc[2], 2)) >= 0.8 && sqrt(pow(acc[3], 2)+ pow(acc[3], 2)+pow(acc[3], 2)) >= 300 ) {
		FALL = 1;
		}
		delay_ms(100);
} 
