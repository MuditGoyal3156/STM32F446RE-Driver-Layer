/*
 * int.h
 *
 *  Created on: Nov 4, 2025
 *      Author: mudit
 */

#ifndef INT_H_
#define INT_H_



#define GPIOA_BASEADDR     0x40020000U
#define GPIOB_BASEADDR     0x40020400U
#define GPIOA_ODRADDR	   (GPIOA_BASEADDR + 0x14U)
#define GPIOB_IDRADDR	   (GPIOB_BASEADDR + 0x10U)


#define RCC_BASEADDR       0x40023800U
#define AHB1ENR_ADDR	   (RCC_BASEADDR + 0x30U)
#define APB2ENR_ADDR	   (RCC_BASEADDR + 0x44U)   // 14th bit for SYSCFG

#define SYSCFG_BASEADDR    0x40013800U
#define NVIC_ISER_BASEADDR 0xE000E100U
#define NVIC_IPR_BASEADDR  0xE000E400

#define IRQ_NO_EXTI9_5		23
#define EXTI_BASEADDR		0x40013C00U

void GPIO(void);
void INT(void);


#endif /* INT_H_ */
