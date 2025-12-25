/*
 * Systick.c
 *
 *  Created on: Dec 25, 2025
 *      Author: mudit
 */


#include "stm32f446xx.h"





void systickDelayMs(int delay)
{
	/*Configure systick*/
	/*Reload with number of clock per millisecond*/
	SysTick->RVR = SYSTICK_LOAD_VAL;

	/*Clear systick current value register*/
	SysTick->CVR = 0;

	/*Enable systick and select internal clock source*/
	SysTick->CSR = CTRL_ENABLE | CTRL_CLKSRC ;

	for(int i=0;i<delay;i++)
	{
		/*Wait until COUNTFLAG is set*/
		while(!(SysTick->CSR & CTRL_COUNTFLAG)){}

	}
	// Disable SysTick
	SysTick->CSR = 0;

}


void systick_1hz_interrupt(void)
{
	/*Reload with number of clock per millisecond*/
	SysTick->RVR = ONE_SEC_LOAD - 1;  //For 1 MS interrupt replace ONE_SEC_LOAD  with  ONE_MS_LOAD

	/*Clear systick current value register*/
	SysTick->CVR = 0;

	/*Enable systick and select internal clock source*/
	SysTick->CSR = CTRL_ENABLE | CTRL_CLKSRC ;

	/*Enable Systick interrupt*/
	SysTick->CSR |= CTRL_TICKINT;
}


/*
 * For main Systick Handler
 static void systick_callback(void)
{
	printf("A second passed");
}

void SysTick_Handler(void)
{
	//Do something..
	systick_callback();
}
 */
