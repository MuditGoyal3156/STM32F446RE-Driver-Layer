/*
 *
 *  Created on: Dec 24, 2025
 *      Author: mudit
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>


#define SLAVE_ADDR  	0x68U
#define MY_ADDR			SLAVE_ADDR

uint8_t commandCode;


//Delay function
void delay(void)
{
	for(uint32_t i = 0 ; i< 250000 ; i++);
}

I2C_Handle_t I2C1Handle;

uint8_t Tx_buff[32]= "STM32 slave mode testing";//limitation of wire library in arduino is 32 bytes


/*
 * Alternate Function Mode : 4
 * PB6 --> SCL
 * PB7 --> SDA

 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIObtn;
	GPIObtn.pGPIOx = GPIOC;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIObtn);
}

int main(void)
{

	GPIO_ButtonInit();

	//Configure the pins
	I2C1_GPIOInits();

	//Initialize the I2C peripheral
	I2C1_Inits();

	//IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV ,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER ,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	//Enable the peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//Enable ACK
	I2C1->CR1 |= (1 << 10);

	while(1){}
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants data
		if(commandCode == 0x51)
		{
			//Send length info to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char *)Tx_buff));
		}
		else if(commandCode == 0x52)
		{
			//Send the contents of Tx_buff
			I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buff[cnt++]);
		}
	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for slave to read
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if(AppEv == I2C_ERROR_AF)
	{
		//This happen only during slave txing
		//Master has sent the NACK. so slave should understand that master doesn't need more data
		commandCode = 0xff; // reset commandcode variable
		cnt = 0;
	}else if(AppEv == I2C_EV_STOP)
	{
		//This happens only during slave reception
		//Master has ended I2C communication with the slave
	}
}
