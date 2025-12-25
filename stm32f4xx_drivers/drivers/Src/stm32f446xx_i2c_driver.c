/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 18, 2025
 *      Author: mudit
 */
#include "stm32f446xx.h"





void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//enable the clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ACK Control bit
	pI2CHandle->pI2Cx->CR1 |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;

	//Configure the FREQ field of CR2
	tempreg=0;
	tempreg |= (RCC_GetPCLK1Value()/1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program device own address(7 bit address mode)
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg &= ~(1 << 15); // to select 7 bit addressing mode
	tempreg |= (1 << 14); // 14 bit should always be 1
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is Standard Mode
		ccr_value = (RCC_GetPCLK1Value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//Mode is Fast Mode
		//Set Bit 15(fast mode)
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);//Set duty cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//Trise config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is Standard Mode
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}else
	{
		//Mode is Fast mode
		tempreg = ((RCC_GetPCLK1Value()*300)/1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{

			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);//(7 bit address + 1 R/W bit)
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteReceiveAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;//(7 bit address + 1 R/W bit)
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;
	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//Clear the ACK bit
				pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);

				//Clear the ADDR Flag(read SR1 and SR2)
				dummyread = pI2CHandle->pI2Cx->SR1;
				dummyread = pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}
		else
		{
			//Clear the ADDR Flag(read SR1 and SR2)
			dummyread = pI2CHandle->pI2Cx->SR1;
			dummyread = pI2CHandle->pI2Cx->SR2;
			(void)dummyread;
		}
	}else
	{
		//Device is in slave mode
		//Clear the ADDR Flag(read SR1 and SR2)
		dummyread = pI2CHandle->pI2Cx->SR1;
		dummyread = pI2CHandle->pI2Cx->SR2;
		(void)dummyread;
	}
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm that start generation is completed by checking the SB flag in SR1
	//Until SB is cleared the SCL will be stretched(pulled to low)
	while(!(pI2CHandle->pI2Cx->SR1 & 0x01));

	//Send Address of the slave with R/W bit set to 0(7 bit address + 1 R/W bit)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//Confirm the address phase is completed by checking the ADDR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & 0x02))
	{
		if(pI2CHandle->pI2Cx->SR1 & (1 << 10))
		    {
		        // 1. Generate STOP
		        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		        // 2. Clear AF flag
		        pI2CHandle->pI2Cx->SR1 &= ~(1 << 10);
		        // 3. Return/Exit to avoid hang
		        return;
		    }
	}

	//Clear the ADDR Flag
	I2C_ClearADDRFlag(pI2CHandle);

	//Send data until the length becomes 0
	while(Len > 0)
	{
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << 7)));
		pI2CHandle->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++;
		Len--;
	}
	//When Len becomes 0 wait for TxE=1 and BTF = 1 before generating the stop condition
	//NOTE: TXE=1, BTF=1 ,means that both SR and DR are Empty and next transmission should begin
	//when BTF=1 SCL will be stretched(pulled to LOW)
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << 7)));//TXE is Set
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << 2)));//BTF is Set

	//Generate the STOP condition and master need to wait for the completion of stop condition
	//Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_NO_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

	//Re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << 10);
	}

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm that the start generation is completed by checking the SB flag in the SR1
	//Until SB is cleared the SCL will be stretched(pulled to low)
	while(!(pI2CHandle->pI2Cx->SR1 & 0x01));

	//Send the address of the slave with r/w =1
	I2C_ExecuteReceiveAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//wait till address phase is completed by checking the ADDR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & 0x02))
	{
		// Check for AF (Acknowledge Failure)
		if(pI2CHandle->pI2Cx->SR1 & (1 << 10))
		{
		   // 1. Generate STOP
		   I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		   // 2. Clear AF flag
		   pI2CHandle->pI2Cx->SR1 &= ~(1 << 10);
		   // 3. Return/Exit to avoid hang
		   return;
		}
	}

	if(Len == 1)//To read 1 byte of data from slave
	{
		//Clear the ACK bit
		pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << 6)));

		//generate STOP condition
		if(Sr == I2C_NO_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data in to buffer
		*pRXBuffer = pI2CHandle->pI2Cx->DR ;

	}

	//Procedure to read data from slave when len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data until LEN becomes zero
		for(uint32_t i = Len; i>0 ; i--)
		{
			//Wait until RxNE becomes 1
			while(!(pI2CHandle->pI2Cx->SR1 & (1 << 6)));

			if(i == 2)
			{
				//Clear the ACK bit
				pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);

				//Generate the STOP Condition
				if(Sr == I2C_NO_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read data from data register in to the buffer
			*pRXBuffer = pI2CHandle->pI2Cx->DR ;

			//increment the buffer address
			pRXBuffer++;

		}
	}

	//Re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << 10);
	}


}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << 0);
	}else
	{
		pI2Cx->CR1 &= ~( 1 << 0);
	}
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority)
{
	//1.Find IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << 10);
	}
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << 10);
	}

}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB flag is set

		//In this block execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteReceiveAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF and TXE both are set
				if(pI2CHandle->TxLen == 0)
				{
					//Generate the stop condition
					if(pI2CHandle->Sr == I2C_NO_SR)
					{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF Flag(1.read the SR1 2.write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000 ;

		//Notify the application that stop is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode(only in master mode)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//Data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//Load the data into DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//Decrement TX length
					pI2CHandle->TxLen--;

					//Increment the buffer address
					pI2CHandle->pTxBuffer++;

				}
			}
		}else
		{
			//Slave
			//Make sure slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxLen == 1)
				{
					//Load DR
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

					//Decrease length
					pI2CHandle->RxLen--;
				}

				if(pI2CHandle->RxLen > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						//Clear the ACK bit
						pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);
					}

					//read data from data register in to the buffer
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR; ;

					//Decrease length
					pI2CHandle->RxLen--;

					//increment the buffer address
					pI2CHandle->pRxBuffer++;

				}
				if(pI2CHandle->RxLen == 0)
				{
					//Close the I2C data reception and notify the application

					//Generate the stop condition
					if(pI2CHandle->Sr == I2C_NO_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//Close I2C RX
					I2C_CloseReceiveData(pI2CHandle);

					//Notify the application
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
				}
			}
		}else
		{
			//Slave
			//Make sure slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


//Check for Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

//Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

//Check for ACK failure  error

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

//Check for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

//Check for Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}else
	{
		//Implement the code to disable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to disable ITEVFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to disable ITERREN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}
