#ifndef UART_H
#define UART_H

/// USART3 -> PB10 (Tx) and PB11(Rx)
/// USART2 -> PA2 (Tx) and PA3(Rx)
/// USART1 -> PA9 (Tx) and PA10(Rx)
	
#include "stm32f10x.h"
#include "gpio.h"
#include "systick.h"

#define BR_9600 0
#define BR_115200 1

#define UART1 1
#define UART2 2
#define UART3 3

void uart_init(unsigned int usart, unsigned short br);
void uart_tx(unsigned int usart, char c);
char uart_rx(unsigned int usart);
void uart_send_msg(unsigned int uart, char str[]);

#endif
