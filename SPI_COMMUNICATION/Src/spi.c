/*
 * spi.c
 *
 *  Created on: Dec 14, 2025
 *      Author: mudit
 */

#include "spi.h"


volatile uint32_t *RCC_APB1ENR = (volatile uint32_t*)RCC_APB1ENR_ADDR;
volatile uint32_t *SPI2_CR1 = (volatile uint32_t*)SPI2_CR1_ADDR;
volatile uint32_t *SPI2_CR2 = (volatile uint32_t*)SPI2_CR2_ADDR;
volatile uint32_t *SPI2_SR = (volatile uint32_t*)SPI2_SR_ADDR;
volatile uint32_t *SPI2_DR = (volatile uint32_t*)SPI2_DR_ADDR;

void SPI2_Clk_EN(void)
{
	*RCC_APB1ENR &= ~( 1U << 14);	//Reset Bit
	*RCC_APB1ENR |= ( 1U << 14);	//Set Bit
}

void SPI2_Init(uint8_t Mode,uint8_t Bus_config, uint8_t Clk_prescaler, uint8_t DFF, uint8_t CPOL, uint8_t CPHA,uint8_t SSM)
{
	//Clock Init
	SPI2_Clk_EN();

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
		SPI_SSIConfig(ENABLE);
	}

}


void SPI2_Data_Send(uint8_t *pTxBuffer,uint8_t Len)
{
	if(pTxBuffer == NULL || Len == 0) return;
	uint8_t dummy_read;
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
		while(!(*SPI2_SR & (1U << 0)));
		//Dummy Read
		dummy_read = *SPI2_DR;
		(void)dummy_read;
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

void SPI_SSIConfig(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		*SPI2_CR1 |= ( 1 << 8);
	}else
	{
		*SPI2_CR1 &= ~( 1 << 8);
	}
}

void SPI_SSOEConfig(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		*SPI2_CR2 |= ( 1 << 2);
	}else
	{
		*SPI2_CR2 &= ~( 1 << 2);
	}
}

void SPI2_IRQInterruptConfig(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//Program ISER1 register
		*NVIC_ISER1 |= ( 1 << ( 37 % 32 ));


	}else{
		//Program ICER1 register
		*NVIC_ICER1 |= ( 1 << ( 37 % 32 ));
	}

}

void SPI2_IRQPriorityConfig(uint32_t IRQPriority)
{
	//1.Find IPR register
	uint8_t iprx = 37 / 4;
	uint8_t iprx_section = 37 % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + 4;
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

void SPI2_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//Check for TXE
	temp1 = *SPI2_SR & (1 << 1);//Check for TXE flag
	temp2 = *SPI2_CR2 & (1 << 7); //Check for TXEIE bit

	if(temp1 & temp2)
	{
		//Handle TXE
		spi2_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE
	temp1 = *SPI2_SR & (1 << 0);//Check for RXNE flag
	temp2 = *SPI2_CR2 & (1 << 6); //Check for RXNEIE bit

	if(temp1 & temp2)
	{
		//Handle RXNE
		spi2_rxne_interrupt_handle(pSPIHandle);
	}
}

void spi2_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if(*SPI2_CR1 & (1 << 11))
	{
		//16 bit DFF
		//1.Load data into the DR
		*SPI2_DR = *((uint16_t*)pSPIHandle->pTxBuffPtr);

		//2. Decrement length twice
		pSPIHandle->TxLen -= 2;

		//3. Increment the data buffer pointer
		pSPIHandle->pTxBuffPtr += 2;
	}
	else
	{
		//8 bit DFF
		//1.Load data into the DR
		*SPI2_DR = *pSPIHandle->pTxBuffPtr;

		//2. Decrement length
		pSPIHandle->TxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pTxBuffPtr++;
	}
		if(!pSPIHandle->TxLen)
		{
			//TxLen is zero,close the SPI communication and inform the application that TX is over
			*SPI2_CR2 &= ~(1 << 7);
			pSPIHandle->pTxBuffPtr = NULL;
			pSPIHandle->TxLen = 0;
			pSPIHandle->State = SPI_DEVICE_FREE	;
			SPI_ApplicationEventCallback();
		}
}
void spi2_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if(*SPI2_CR1 & (1 << 11))
	{
		//16 bit DFF
		//1.Load data from the DR
		*((uint16_t*)pSPIHandle->pRxBuffPtr) = *SPI2_DR  ;

		//2. Decrement length twice
		pSPIHandle->RxLen -=2;


		//3. Increment the data buffer pointer
		pSPIHandle->pRxBuffPtr += 2;
	}
	else
	{
		//8 bit DFF
		//1.Load data into the DR
		*pSPIHandle->pRxBuffPtr = *SPI2_DR ;

		//2. Decrement length
		pSPIHandle->RxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pRxBuffPtr++;
	}
		if(!pSPIHandle->RxLen)
		{
			//RxLen is zero,close the SPI communication and inform the application that TX is over
			*SPI2_CR2 &= ~(1 << 6);
			pSPIHandle->pRxBuffPtr = NULL;
			pSPIHandle->RxLen = 0;
			pSPIHandle->State = SPI_DEVICE_FREE	;
			SPI_ApplicationEventCallback();
		}
}

__attribute__((weak)) void SPI_ApplicationEventCallback()
{
	//This is a weak implementation, the application may override this function
}







