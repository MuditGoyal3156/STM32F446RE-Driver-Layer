/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Dec 13, 2025
 *      Author: mudit
 */
#include "stm32f446xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{

			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx , ENABLE);

	//Configure SPI_CR1 register
	uint32_t tempreg=0;

	//1.Configure the device mode
	tempreg |=pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2.Configure the BusConfig
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1<< 15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1<< 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1<< 15);
		//RXONLY bit must be set
		tempreg |= ( 1<< 10);
	}
	//3. Configure the spi serial clock speed (baud rate)
	tempreg |=pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. Configure the DFF
	tempreg |=pSPIHandle->SPIConfig.SPI_DFF << 11;

	//5. Configure the CPOL
	tempreg |=pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. Configure the CPHA
	tempreg |=pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->CR1 = tempreg;


}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait for TXE flag to set
		while (!(pSPIx->SR & (1 << 1)));

		//2.Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << 11))
		{
			//16 bit DFF
			//1.Load data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);

			//2. Decrement length twice
			Len--;
			Len--;

			//3. Increment the data buffer pointer
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			//1.Load data into the DR
			pSPIx->DR = *pTxBuffer;

			//2. Decrement length
			Len--;

			//3. Increment the data buffer pointer
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t Len)
{

}


void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
