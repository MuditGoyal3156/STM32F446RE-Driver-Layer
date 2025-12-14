/*
 * spi.c
 *
 *  Created on: Dec 14, 2025
 *      Author: mudit
 */

#include "spi.h"


uint32_t *RCC_APB1ENR = (uint32_t*)RCC_APB1ENR_ADDR;
uint32_t *SPI2_CR1 = (uint32_t*)SPI2_CR1_ADDR;
uint32_t *SPI2_CR2 = (uint32_t*)SPI2_CR2_ADDR;
uint32_t *SPI2_SR = (uint32_t*)SPI2_SR_ADDR;
uint32_t *SPI2_DR = (uint32_t*)SPI2_DR_ADDR;

void SPI2_Clk_EN(void)
{
	*RCC_APB1ENR &= ~( 1U << 14);	//Reset Bit
	*RCC_APB1ENR |= ( 1U << 14);	//Set Bit
}

void SPI2_Init(uint8_t Mode,uint8_t Bus_config, uint8_t Clk_prescaler, uint8_t DFF, uint8_t CPOL, uint8_t CPHA,uint8_t SSM)
{
	//1. Master or Slave Mode
	*SPI2_CR1 &= ~( 1U << 2);	//Reset Bit
	*SPI2_CR1 |= ( Mode << 2);	//Set Bit


	//2. Bus Configuration(FD,HD,Simplex)
	if(Bus_config == SPI_BUS_CONFIG_FD)
	{
		*SPI2_CR1 &= ~( 1U << 15);	//Reset BIDIMODE Bit
		*SPI2_CR1 &= ~(1U << 10); // RXONLY = 0

	}else if(Bus_config == SPI_BUS_CONFIG_HD_RX){
		*SPI2_CR1 &= ~( 1U << 15);	//Reset Bit
		*SPI2_CR1 |= ( 1U << 15);	//Set BIDIMODE Bit
		*SPI2_CR1 &= ~(1U << 10); // RXONLY = 0
		//Reset BIDIOE bit
		*SPI2_CR1 &= ~( 1U << 14);	//Reset Bit

	}else if(Bus_config == SPI_BUS_CONFIG_HD_TX){
		*SPI2_CR1 &= ~( 1U << 15);	//Reset BIDIMODE Bit
		*SPI2_CR1 |= ( 1U << 15);	//Set BIDIMODE Bit
		*SPI2_CR1 &= ~(1U << 10); // RXONLY = 0

		//Set BIDIOE bit
		*SPI2_CR1 &= ~( 1U << 14);	//Reset Bit
		*SPI2_CR1 |= ( 1U << 14);	//Set Bit


	}else if(Bus_config == SPI_BUS_CONFIG_SIMPLEX_RXONLY){

		*SPI2_CR1 &= ~( 1U << 15);	//Reset BIDIMODE Bit
		//Enable RXONLY mode
		*SPI2_CR1 &= ~( 1U << 10);	//Reset Bit
		*SPI2_CR1 |= ( 1U << 10);	//Set Bit

	}

	//3. Clock Pre-Scaler
	*SPI2_CR1 &= ~(7U << 3);	//Reset Bits
	*SPI2_CR1 |= Clk_prescaler << 3; //Set Bits

	//4. Data Frame Format
	*SPI2_CR1 &= ~(1U << 11);	//Reset Bit
	*SPI2_CR1 |=  DFF << 11; //Set Bit

	//5. Clock Polarity
	*SPI2_CR1 &= ~(1U << 1);	//Reset Bit
	*SPI2_CR1 |=  CPOL << 1; //Set Bit

	//6. Clock Phase
	*SPI2_CR1 &= ~(1U << 0);	//Reset Bit
	*SPI2_CR1 |=  CPHA << 0; //Set Bit

	//7.SSM
	//SSM bit in CR1
	*SPI2_CR1 &= ~(1U << 9);	//Reset Bit
	*SPI2_CR1 |=  SSM << 9; //Set Bit
	if(Mode == SPI_DEVICE_MODE_MASTER && SSM == SPI_SSM_EN)
	{
		*SPI2_CR1 |= (1U << 8);//SSI bit in CR1
	}

}


void SPI2_Data_Send(uint8_t *pTxBuffer,uint8_t Len)
{
	if(pTxBuffer == NULL || Len == 0) return;

	while(Len > 0)
	{
		//1. Wait for TXE flag to set
		while(!(*SPI2_SR & (1U << 1)));

		//If DFF is 16 bit
		if(*SPI2_CR1 & (1U << 11))
		{
			*SPI2_DR = *((uint16_t *)pTxBuffer);//Load Data register
			Len -=2;		//Decrement Length
			pTxBuffer += 2;//Increment buffer
		}

		//If DFF is 8 bit
		else
		{
			*SPI2_DR = *pTxBuffer;//Load Data register
			Len --;		//Decrement Length
			pTxBuffer++;//Increment buffer
		}
	}
	//Wait until BSY = 0 (transfer complete)
	while(*SPI2_SR & (1U << 7));
}

void SPI2_Data_Read(uint8_t *pRxBuffer,uint8_t Len)
{
	uint8_t dummy = 0xFF;

	while(Len > 0)
	{
		//1. Wait until TXE = 1
		while(!(*SPI2_SR & (1U << 1)));

		//2. Send dummy byte to generate clock
		*SPI2_DR = dummy;

		//3. Wait for RXNE flag to set
		while(!(*SPI2_SR & (1U << 0)));

		//4. Read received data
		//If DFF is 16 bit
		if(*SPI2_CR1 & (1U << 11))
		{
			 *((uint16_t *)pRxBuffer) = *SPI2_DR;//Load buffer
			Len -=2;		//Decrement Length
			pRxBuffer += 2;//Increment buffer
		}
		//If DFF is 8 bit
		else
		{
			*pRxBuffer = *SPI2_DR;//Load buffer
			Len --;		//Decrement Length
			pRxBuffer++;//Increment buffer
		}
	}

	//Wait until BSY = 0 (transfer complete)
	while(*SPI2_SR & (1U << 7));
}
void SPI_EN(void)
{
	//SPE bit in C1
	*SPI2_CR1 &= ~(1U << 6);	//Reset Bit
	*SPI2_CR1 |=  (1U << 6); //Set Bit
}
void SPI_DI(void)
{
	//Wait until BSY = 0
	while(*SPI2_SR & (1U << 7));

	//SPE bit in C1
	*SPI2_CR1 &= ~(1U << 6);	//Reset Bit
}
