/*
 * 004led_button_int.c
 *
 *  Created on: Nov 1, 2025
 *      Author: mudit
 */


#include "stm32f446xx.h"
#include<string.h>

void delay(void)
{
	for(uint32_t i = 0 ; i< 250000 ; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOled,GPIObtn;
	memset(&GPIOled,0,sizeof(GPIOled));		//initialize everything in the structure to zero
	memset(&GPIObtn,0,sizeof(GPIObtn));		//initialize everything in the structure to zero
	GPIOled.pGPIOx = GPIOA;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOled);


	GPIObtn.pGPIOx = GPIOC;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GPIObtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);

	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
}

