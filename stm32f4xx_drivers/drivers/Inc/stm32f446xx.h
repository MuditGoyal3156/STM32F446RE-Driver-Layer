/*
 * stm32f446xx.h
 *
 *  Created on: Oct 29, 2025
 *      Author: mudit
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>
#include <stddef.h>
#define __vo		volatile

/*
 * Base Address of SysTick
 */
#define SYSTICK_BASE_ADDR	((__vo uint32_t*)0xE000E010)
/*
 * NVIC ISERx register Addresses
 */
#define NVIC_ISER0		( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1		( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2		( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3		( (__vo uint32_t*)0xE000E10C )

/*
 * NVIC ICERx register Addresses
 */
#define NVIC_ICER0		( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1		( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2		( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3		( (__vo uint32_t*)0XE000E18c )

/*
 *Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*) 0xE000E400)

/*
 * Number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED		4
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR     0x08000000U
#define SRAM1_BASEADDR     0x20000000U   //112KB
#define SRAM2_BASEADDR     0x20001C00U   //16KB
#define ROM                0x1FFF0000U
#define SRAM               SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE        0x40000000U
#define AHB1PERIPH_BASE    0x40020000U
#define AHB2PERIPH_BASE    0x50000000U
#define APB1PERIPH_BASE    PERIPH_BASE
#define APB2PERIPH_BASE    0x40010000U

/*
 * base addresses of peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR     0x40020000U
#define GPIOB_BASEADDR     0x40020400U
#define GPIOC_BASEADDR     0x40020800U
#define GPIOD_BASEADDR     0x40020C00U
#define GPIOE_BASEADDR     0x40021000U
#define GPIOF_BASEADDR     0x40021400U
#define GPIOG_BASEADDR     0x40021800U
#define GPIOH_BASEADDR     0x40021C00U

#define RCC_BASEADDR       0x40023800U


/*
 * base addresses of peripherals on APB1 bus
 */

#define I2C1_BASEADDR      0x40005400U
#define I2C2_BASEADDR      0x40005800U
#define I2C3_BASEADDR      0x40005C00U

#define USART2_BASEADDR    0x40004400U
#define USART3_BASEADDR    0x40004800U
#define UART4_BASEADDR     0x40004C00U
#define UART5_BASEADDR     0x40005000U

#define SPI2_BASEADDR      0x40003800U
#define SPI3_BASEADDR      0x40003C00U


/*
 * base addresses of peripherals on APB2 bus
 */

#define SPI1_BASEADDR      0x40013000U
#define SPI4_BASEADDR      0x40013400U
#define USART1_BASEADDR    0x40011000U
#define USART6_BASEADDR    0x40011400U
#define SYSCFG_BASEADDR    0x40013800U
#define EXTI_BASEADDR      0x40013C00U


typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR ;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;


typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t Reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t Reserved2;
	uint32_t Reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t Reserved4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t Reserved7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t Reserved8;
	uint32_t Reserved9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t Reserved10;
	uint32_t Reserved11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t   		  Reserved1[2];
	volatile uint32_t CMPCR;
	uint32_t 		  Reserved2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;


typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR ;
	volatile uint32_t TXCRCR ;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;

}USART_RegDef_t;


typedef struct
{
	volatile uint32_t CSR;
	volatile uint32_t RVR;
	volatile uint32_t CVR;
	volatile uint32_t CALIB;
}SYSTICK_RegDef_t;


/*
 * peripheral definition ( Peripheral base addresses typecasted to xxx_RegDef_t)
*/

#define GPIOA      ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB      ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF      ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG      ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH      ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC        ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI	   ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG	   ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1      ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2      ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3      ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4      ((SPI_RegDef_t *)SPI4_BASEADDR)

#define I2C1      ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2      ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3      ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1      ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2      ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3      ((USART_RegDef_t *)USART3_BASEADDR)
#define UART4       ((USART_RegDef_t *)UART4_BASEADDR)
#define UART5       ((USART_RegDef_t *)UART5_BASEADDR)
#define USART6      ((USART_RegDef_t *)USART6_BASEADDR)

#define SysTick      ((SYSTICK_RegDef_t *)SYSTICK_BASE_ADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()  ( RCC->AHB1ENR |= ( 1 << 7) )


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 23))


/*
 * Clock Enable Macros for 	SPIx peripherals
 */


#define SPI1_PCLK_EN()  (RCC->APB2ENR |= ( 1 << 12))
#define SPI2_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN()  (RCC->APB2ENR |= ( 1 << 13))


/*
 * Clock Enable Macros for 	USARTx peripherals
 */

#define USART1_PCLK_EN()  (RCC->APB2ENR |= ( 1 << 4))
#define USART2_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 17))
#define USART3_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 18))
#define UART4_PCLK_EN()   (RCC->APB1ENR |= ( 1 << 19))
#define UART5_PCLK_EN()   (RCC->APB1ENR |= ( 1 << 20))
#define USART6_PCLK_EN()  (RCC->APB2ENR |= ( 1 << 5))


/*
 * Clock Enable Macros for 	SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= ( 1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 0) )
#define GPIOB_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 1) )
#define GPIOC_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 2) )
#define GPIOD_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 3) )
#define GPIOE_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 4) )
#define GPIOF_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 5) )
#define GPIOG_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 6) )
#define GPIOH_PCLK_DI()  ( RCC->AHB1ENR &= ~( 1 << 7) )


/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 23))


/*
 * Clock Disable Macros for SPIx peripherals
 */


#define SPI1_PCLK_DI()  (RCC->APB2ENR &= ~( 1 << 12))
#define SPI2_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 15))
#define SPI4_PCLK_DI()  (RCC->APB2ENR &= ~( 1 << 13))


/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~( 1 << 4))
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 17))
#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~( 1 << 18))
#define UART4_PCLK_DI()   (RCC->APB1ENR &= ~( 1 << 19))
#define UART5_PCLK_DI()   (RCC->APB1ENR &= ~( 1 << 20))
#define USART6_PCLK_DI()  (RCC->APB2ENR &= ~( 1 << 5))


/*
 * Clock Disable Macros for 	SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~( 1 << 14))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 0) );	( RCC->AHB1RSTR &= ~( 1 << 0) );	}while(0)	//no semicolon after while because semicolon already there in function call
#define GPIOB_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 1) );	( RCC->AHB1RSTR &= ~( 1 << 1) );	}while(0)
#define GPIOC_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 2) );	( RCC->AHB1RSTR &= ~( 1 << 2) );	}while(0)
#define GPIOD_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 3) );	( RCC->AHB1RSTR &= ~( 1 << 3) );	}while(0)
#define GPIOE_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 4) );	( RCC->AHB1RSTR &= ~( 1 << 4) );	}while(0)
#define GPIOF_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 5) );	( RCC->AHB1RSTR &= ~( 1 << 5) );	}while(0)
#define GPIOG_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 6) );	( RCC->AHB1RSTR &= ~( 1 << 6) );	}while(0)
#define GPIOH_REG_RESET()  do{( RCC->AHB1RSTR |= ( 1 << 7) );	( RCC->AHB1RSTR &= ~( 1 << 7) );	}while(0)

#define GPIO_BASEADDR_TO_CODE(x)		((x==GPIOA) ? 0 : \
										 (x==GPIOB) ? 1 : \
										 (x==GPIOC) ? 2 : \
										 (x==GPIOD) ? 3 : \
										 (x==GPIOE) ? 4 : \
										 (x==GPIOF) ? 5 : \
										 (x==GPIOG) ? 6 : \
										 (x==GPIOH) ? 7 : 0)

/*
 * IRQ numbers of STM32F446RE MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_SPI1 			36
#define IRQ_NO_SPI2 			36
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI3 			51
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73
#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71
/*
 * IRQ Priority
 */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()  do{( RCC->APB2RSTR |= ( 1 << 12) );	( RCC->APB2RSTR &= ~( 1 << 12) );	}while(0)	//no semicolon after while because semicolon already there in function call
#define SPI2_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 14) );	( RCC->APB1RSTR &= ~( 1 << 14) );	}while(0)
#define SPI3_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 15) );	( RCC->APB1RSTR &= ~( 1 << 15) );	}while(0)
#define SPI4_REG_RESET()  do{( RCC->APB2RSTR |= ( 1 << 13) );	( RCC->APB2RSTR &= ~( 1 << 13) );	}while(0)


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 21) );	( RCC->APB1RSTR &= ~( 1 << 21) );	}while(0)	//no semicolon after while because semicolon already there in function call
#define I2C2_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 22) );	( RCC->APB1RSTR &= ~( 1 << 22) );	}while(0)
#define I2C3_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 23) );	( RCC->APB1RSTR &= ~( 1 << 23) );	}while(0)


/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()  do{( RCC->APB2RSTR |= ( 1 << 4) );	( RCC->APB2RSTR &= ~( 1 << 4) );	}while(0)
#define USART2_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 17) );	( RCC->APB1RSTR &= ~( 1 << 17) );	}while(0)
#define USART3_REG_RESET()  do{( RCC->APB1RSTR |= ( 1 << 18) );	( RCC->APB1RSTR &= ~( 1 << 18) );	}while(0)
#define UART4_REG_RESET()   do{( RCC->APB1RSTR |= ( 1 << 19) );	( RCC->APB1RSTR &= ~( 1 << 19) );	}while(0)
#define UART5_REG_RESET()   do{( RCC->APB1RSTR |= ( 1 << 20) );	( RCC->APB1RSTR &= ~( 1 << 20) );	}while(0)
#define USART6_REG_RESET()  do{( RCC->APB2RSTR |= ( 1 << 5) );	( RCC->APB2RSTR &= ~( 1 << 5) );	}while(0)

/*
 * Bit position definition macros for I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

/*
 * Bit position definition macros for I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

/*
 * Bit position definition macros for I2C_OAR1
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMODE		15

/*
 * Bit position definition macros for I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

/*
 * Bit position definition macros for I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

/*
 * Bit position definition macros for I2C_CCR
 */
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15


/*
 * Bit position definition macros for USART_SR
 */
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

/*
 * Bit position definition macros for USART_BRR
 */
#define USART_BRR_DIV_FRAC30	0
#define USART_BRR_DIV_MANT110	4

/*
 * Bit position definition macros for USART_CR1
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * Bit position definition macros for USART_CR2
 */
#define USART_CR2_ADD30			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBIE			6
#define USART_CR2_LBCK			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP10		12
#define USART_CR2_LINEN			14

/*
 * Bit position definition macros for USART_CR3
 */
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11
/*
 * Bit position definition macros for SYST_CSR
 */
#define CTRL_ENABLE        (1U << 0)
#define CTRL_TICKINT       (1U << 1)
#define CTRL_CLKSRC        (1U << 2)
#define CTRL_COUNTFLAG     (1U << 16)

//some generic macros
#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_uart_driver.h"
#include "RCC_Peripheral.h"
#include "Print_on_Serial_terminal.h"
#include "Systick.h"
#endif /* INC_STM32F446XX_H_ */
