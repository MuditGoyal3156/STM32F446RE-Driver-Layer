/*
 * spi.h
 *
 *  Created on: Dec 14, 2025
 *      Author: mudit
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <stddef.h>

#define RCC_BASEADDR			0x40023800U
#define SPI2_BASEADDR			0x40003800U
#define RCC_APB1ENR_ADDR		RCC_BASEADDR + 0x40U
#define SPI2_CR1_ADDR			SPI2_BASEADDR + 0x00U
#define SPI2_CR2_ADDR			SPI2_BASEADDR + 0x04U
#define SPI2_SR_ADDR			SPI2_BASEADDR + 0x08U
#define SPI2_DR_ADDR			SPI2_BASEADDR + 0x0CU

/*
 * Macros for master or slave
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*
 * Macros for Bus Config
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD_RX				2
#define SPI_BUS_CONFIG_HD_TX				3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

/*
 * Macros for Prescaler
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * Macros for DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * Macros for CPOL
 */
#define SPI_CPOL_HIGH 		 1
#define SPI_CPOL_LOW		 0

/*
 * Macros for CPHA
 */
#define SPI_CPHA_HIGH 		 1
#define SPI_CPHA_LOW		 0

/*
 * Macros for SSM
 */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1


/**
 ******************************************************************************
 *                       Function Prototypes
 ******************************************************************************
 */

// Clock Enable
void SPI2_Clk_EN(void);


//SPI Initialization
void SPI2_Init(uint8_t Mode,uint8_t Bus_config, uint8_t Clk_prescaler, uint8_t DFF, uint8_t CPOL, uint8_t CPHA, uint8_t SSM);

//Data Transmission and Reception(polling)
void SPI2_Data_Send(uint8_t *pTxBuffer,uint8_t Len);
void SPI2_Data_Read(uint8_t *pRxBuffer,uint8_t Len);

//Enable or Disable SPI
void SPI_EN(void);
void SPI_DI(void);
#endif /* SPI_H_ */
