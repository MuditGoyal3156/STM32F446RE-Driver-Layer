/*
 * Print_on_Serial_terminal.c
 *
 *  Created on: Dec 25, 2025
 *      Author: mudit
 */

#include "stm32f446xx.h"

/*
 * Call uart2_tx_init() in main
 * in printf("......\r\n");
 */

int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void uart2_tx_init(void)
{
	/*Configure UART GPIO pin*/
	/*Enable clock access to GPIOA*/
	GPIOA_PCLK_EN();

	/*Set PA2 mode to Alternate function mode*/
	GPIOA->MODER &= ~(1U << 4);
	GPIOA->MODER |= (1U << 5);

	/*Set PA2 alternate function type to UART_TX (AF07)*/
	GPIOA->AFR[0] &= ~(0xF << 8);   // clear AF bits
	GPIOA->AFR[0] |=  (7U  << 8);


	/*Configure UART module*/
	/*Enable clock access to uart2*/
	USART2_PCLK_EN();

	/*Configure baudrate*/
	USART_SetBaudRate(USART2, UART_BAUDRATE);

	/*Configure the transfer direction*/
	USART2->CR1 |= (1U << 3);

	/*Enable uart module*/
	USART2->CR1 |=(1U << 13);
}

void uart2_write(int ch)
{
	 /*Make sure transmit data register is empty*/
	while(!(USART2->SR & (1 << USART_SR_TXE))){}
	/*Write to transmit data register*/
	USART2->DR = (ch & 0xFF);
}


