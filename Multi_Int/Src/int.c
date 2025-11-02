/*
 * int.c
 *
 *  Created on: Nov 2, 2025
 *      Author: mudit
 */


#include <stdint.h>
#include "int.h"

uint32_t* gpioa = (uint32_t*) GPIOA_BASEADDR ;
uint32_t* gpiob = (uint32_t*) GPIOB_BASEADDR ;
uint32_t* gpioc = (uint32_t*) GPIOC_BASEADDR ;
uint32_t* gpioclken = ( uint32_t* )AHB1ENR_ADDR;
uint32_t* gpioaODR = (uint32_t*)GPIOA_ODRADDR;

uint32_t* GPIOA_MODER = (uint32_t*) GPIOA_BASEADDR ;
uint32_t* GPIOB_MODER = (uint32_t*) GPIOB_BASEADDR ;
uint32_t* GPIOC_MODER = (uint32_t*) GPIOC_BASEADDR ;

uint32_t* GPIOA_PUPDR = (uint32_t*) ( GPIOA_BASEADDR + 0x0CU );
uint32_t* GPIOB_PUPDR = (uint32_t*) ( GPIOB_BASEADDR + 0x0CU );
uint32_t* GPIOC_PUPDR = (uint32_t*) ( GPIOC_BASEADDR + 0x0CU );


uint32_t* SYSCFG_EXTICR2 = (uint32_t*) ( SYSCFG_BASEADDR + 0x0CU );
uint32_t* SYSCFG_EXTICR3 = (uint32_t*) ( SYSCFG_BASEADDR + 0x10U );


uint32_t* NVIC_ISER0 = (uint32_t*) NVIC_ISER_BASEADDR ;


uint32_t* NVIC_IPR5 = (uint32_t*) ( NVIC_IPR_BASEADDR + 0x14U );

uint32_t* EXTI_FTSR = (uint32_t*) ( EXTI_BASEADDR + 0x0CU );
uint32_t* EXTI_IMR = (uint32_t*) EXTI_BASEADDR ;

uint32_t* SYSCFG_CLK_EN = ( uint32_t* )APB2ENR_ADDR;


void GPIO(void)
{
	 /*
	  * Clock enable for GPIOA,GPIOB and GPIOC
	  */
	*gpioclken |= ( 1U << 0 ) | ( 1U << 1 ) | ( 1U << 2 ) ;//setting

	/*
	 * Mode selection
	 */

	*GPIOA_MODER |= ( 1U << 10 ); //PA5 output
	*GPIOA_MODER &= ~( 3U << 14 ); //PA7 input
	*GPIOB_MODER &= ~( 3U << 12 ); //PB6 input
	*GPIOC_MODER &= ~( 3U << 16 ); //PC8 input

	/*
	 * Pull-up
	 */
	*GPIOA_PUPDR |= ( 1U << 14 ); //PA7 Pull-up
	*GPIOB_PUPDR |= ( 1U << 12 ); //PB6 Pull-up
	*GPIOC_PUPDR |= ( 1U << 16 ); //PC8 Pull-up
}

void INT(void)
{
	/*
	 * SYSCFG Clk EN
	 */
	*SYSCFG_CLK_EN |= ( 1U << 14 );

	/*
	 * falling edge trigger
	 */
	*EXTI_FTSR |= ( 1U << 6) | ( 1U << 7) | ( 1U << 8);

	/*
	* Port Code in SYSCFG_EXTICRx registers
	*/
	*SYSCFG_EXTICR2 &= ~(0xF << 8);
	*SYSCFG_EXTICR2 |= (0x1 << 8);    // 0001 = Port B

	*SYSCFG_EXTICR2 &= ~(0xF << 12);
	*SYSCFG_EXTICR2 |= (0x0 << 12);   // 0000 = Port A

	*SYSCFG_EXTICR3 &= ~(0xF << 0);
	*SYSCFG_EXTICR3 |= (0x2 << 0);    // 0010 = Port C

	/*
	 * IMR enable
	 */
	*EXTI_IMR |= ( 1U << 6) | ( 1U << 7) | ( 1U << 8);

	/*
	 * interrupt set enable
	 */
	*NVIC_ISER0 |=( 1U << IRQ_NO_EXTI9_5);

	/*
	 * Setting Priority
	 */
	*NVIC_IPR5 |= (0x08 << 20);

}
void toggle_LED(void)
{
    *gpioaODR ^= (1 << 5);
}

void LED_ON(void)
{
	*gpioaODR |= (1 << 5);
}

void LED_OFF(void)
{
	*gpioaODR &= ~(1 << 5);
}
