/*
 * int.c
 *
 *  Created on: Nov 4, 2025
 *      Author: mudit
 */


#include <stdint.h>
#include "int.h"

uint32_t* GPIOA = (uint32_t*) GPIOA_BASEADDR ;
uint32_t* GPIOB = (uint32_t*) GPIOB_BASEADDR ;

uint32_t* GPIO_CLK_EN = ( uint32_t* )AHB1ENR_ADDR;
uint32_t* GPIOA_ODR = (uint32_t*)GPIOA_ODRADDR;

uint32_t* GPIOA_MODER = (uint32_t*) ( GPIOA_BASEADDR + 0x00U );
uint32_t* GPIOB_MODER = (uint32_t*) ( GPIOB_BASEADDR + 0x00U );


uint32_t* GPIOA_PUPDR = (uint32_t*) ( GPIOA_BASEADDR + 0x0CU );
uint32_t* GPIOB_PUPDR = (uint32_t*) ( GPIOB_BASEADDR + 0x0CU );



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
	  * Clock enable for GPIOA and GPIOB
	  */
	*GPIO_CLK_EN |= ( 1U << 0 ) | ( 1U << 1 )  ;

	/*
	 * Mode selection
	 */

	//Rows
	*GPIOA_MODER |= ( 1U << 10 ); //PA5 output
	*GPIOA_MODER |= ( 1U << 12 ); //PA6 output
	*GPIOA_MODER |= ( 1U << 14 ); //PA7 output
	*GPIOA_MODER |= ( 1U << 16 ); //PA8 output

	//Columns
	*GPIOB_MODER &= ~( 3U << 12 ); //PB6 input
	*GPIOB_MODER &= ~( 3U << 14 ); //PB7 input
	*GPIOB_MODER &= ~( 3U << 16 ); //PB8 input
	*GPIOB_MODER &= ~( 3U << 18 ); //PB9 input


	/*
	 * Pull-up
	 */

	*GPIOB_PUPDR |= ( 1U << 12 ); //PB6 Pull-up
	*GPIOB_PUPDR |= ( 1U << 14 ); //PB7 Pull-up
	*GPIOB_PUPDR |= ( 1U << 16 ); //PB8 Pull-up
	*GPIOB_PUPDR |= ( 1U << 18 ); //PB9 Pull-up

	/*
	 * Setting Rows as HIGH
	 */

	*GPIOA_ODR |= (1 << 5); //PA5 HIGH
	*GPIOA_ODR |= (1 << 6); //PA6 HIGH
	*GPIOA_ODR |= (1 << 7); //PA7 HIGH
	*GPIOA_ODR |= (1 << 8); //PA8 HIGH
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

	*EXTI_FTSR |= ( 1U << 6) | ( 1U << 7) | ( 1U << 8) | ( 1U << 9);

	/*
	* Port Code in SYSCFG_EXTICRx registers
	*/
	*SYSCFG_EXTICR2 &= ~(0xF << 8);
	*SYSCFG_EXTICR2 |= (0x1 << 8);    // 0001 = Port B

	*SYSCFG_EXTICR2 &= ~(0xF << 12);
	*SYSCFG_EXTICR2 |= (0x1 << 12);   // 0001 = Port B

	*SYSCFG_EXTICR3 &= ~(0xF << 0);
	*SYSCFG_EXTICR3 |= (0x1 << 0);    // 0001 = Port B

	*SYSCFG_EXTICR3 &= ~(0xF << 4);
	*SYSCFG_EXTICR3 |= (0x1 << 4);    // 0001 = Port B


	/*
	 * IMR enable
	 */
	*EXTI_IMR |= ( 1U << 6) | ( 1U << 7) | ( 1U << 8) | ( 1U << 9);

	/*
	 * interrupt set enable
	 */
	*NVIC_ISER0 |=( 1U << IRQ_NO_EXTI9_5);

	/*
	 * Setting Priority
	 */
	*NVIC_IPR5 |= (0x08 << 24);

}


