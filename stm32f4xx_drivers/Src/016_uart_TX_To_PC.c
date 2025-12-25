/*
 * 016_uart_TX_To_PC.c
 *
 *  Created on: Dec 25, 2025
 *      Author: mudit
 */



#include<stdio.h>
#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
{
	uart2_tx_init();
	while(1)
	{
		printf("Hello World \r\n");
		delay();
	}
}



