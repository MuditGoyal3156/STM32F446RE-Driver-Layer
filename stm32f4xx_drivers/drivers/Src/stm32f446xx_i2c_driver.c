/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 18, 2025
 *      Author: mudit
 */
#include "stm32f446xx.h"

uint16_t AHB_PreScalar[8]={2,4,8,16,64,128,256,512};
uint8_t APB1_PreScalar[4]={2,4,8,16};

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}




uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksource,temp,ahbp,apb1p;
	clksource = ((RCC->CFGR >> 2) & 0x3);
	if(clksource == 0)
	{
		SystemClk = 16000000;
	}else if(clksource == 1)
	{
		SystemClk = 8000000;
	}if(clksource == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp =1;
	}else
	{
		ahbp = AHB_PreScalar[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1p =1;
	}else
	{
		apb1p = APB1_PreScalar[temp-4];
	}

	pclk1 = (SystemClk/ahbp)/apb1p;
	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//enable the clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ACK Control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ field of CR2
	tempreg=0;
	tempreg |= (RCC_GetPCLK1Value()/1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program device own address(7 bit address mode)
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

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
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
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;
	(void)dummyread;
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
	while(!(pI2CHandle->pI2Cx->SR1 & 0x02));

	//Clear the ADDR Flag
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
	while(!(pI2CHandle->pI2Cx->SR1 & 0x02));

	if(Len == 1)//To read 1 byte of data from slave
	{
		//Clear the ACK bit
		pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
