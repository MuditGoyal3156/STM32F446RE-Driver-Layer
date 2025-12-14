/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Dec 13, 2025
 *      Author: mudit
 */
#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

	//7. Configure the SSM
	tempreg |=pSPIHandle->SPIConfig.SPI_SSM << 9;

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
			pTxBuffer += 2;
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
	while(Len > 0)
	{
		//1. Wait for RXNE flag to set
		while (!(pSPIx->SR & (1 << 0)));

		//2.Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << 11))
		{
			//16 bit DFF
			//1.Read DR
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;

			//2. Decrement length twice
			Len--;
			Len--;

			//3. Increment the data buffer pointer
			pRxBuffer += 2;
		}
		else
		{
			//8 bit DFF
			//1.Read DR
			*pRxBuffer = pSPIx->DR ;

			//2. Decrement length
			Len--;

			//3. Increment the data buffer pointer
			pRxBuffer++;
		}
	}

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << 6);
	}else
	{
		pSPIx->CR1 &= ~( 1 << 6);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= ( 1 << 8);
		}else
		{
			pSPIx->CR1 &= ~( 1 << 8);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR2 |= ( 1 << 2);
		}else
		{
			pSPIx->CR2 &= ~( 1 << 2);
		}
}
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ));

		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ) );

		}
	}else
	{
		if(IRQNumber <=31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ));

		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	}

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority)
{
	//1.Find IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//Check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << 1);//Check for TXE flag
	temp2 = pHandle->pSPIx->CR2 & (1 << 7); //Check for TXEIE bit

	if(temp1 & temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << 0);//Check for RXNE flag
	temp2 = pHandle->pSPIx->CR2 & (1 << 6); //Check for RXNEIE bit

	if(temp1 & temp2)
	{
		//Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check for overrun error
	temp1 = pHandle->pSPIx->SR & (1 << 6);//Check for OVR flag
	temp2 = pHandle->pSPIx->CR2 & (1 << 5); //Check for ERRIE bit

	if(temp1 & temp2)
	{
		//Handle overrun error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len info in some global variable.
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << 7 );

	}

	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len info in some global variable.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in reception so that no other code can take over same SPI peripheral until transmission is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable RXNEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << 6 );

	}

	return state;

}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		//16 bit DFF
		//1.Load data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);

		//2. Decrement length twice
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pTxBuffer += 2;
	}
	else
	{
		//8 bit DFF
		//1.Load data into the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;

		//2. Decrement length
		pSPIHandle->TxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pTxBuffer++;
	}
		if(!pSPIHandle->TxLen)
		{
			//TxLen is zero,close the SPI communication and inform the application that TX is over
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		//16 bit DFF
		//1.Load data into the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR  ;

		//2. Decrement length twice
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pRxBuffer += 2;
	}
	else
	{
		//8 bit DFF
		//1.Load data into the DR
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR ;

		//2. Decrement length
		pSPIHandle->RxLen--;

		//3. Increment the data buffer pointer
		pSPIHandle->pRxBuffer++;
	}
		if(!pSPIHandle->RxLen)
		{
			//TxLen is zero,close the SPI communication and inform the application that TX is over
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//Inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << 7);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << 6);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation, the application may override this function
}
