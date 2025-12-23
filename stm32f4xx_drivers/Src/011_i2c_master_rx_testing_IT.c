/*
 * 009i2c_master_tx_testing.c
 *
 *  Created on: Dec 22, 2025
 *      Author: mudit
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

uint8_t rxCmplt = RESET;
#define MY_ADDR			0x61U
#define SLAVE_ADDR  	0x68U

//Delay function
void delay(void)
{
	for(uint32_t i = 0 ; i< 250000 ; i++);
}

I2C_Handle_t I2C1Handle;

uint8_t rcv_buff[32];


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

	initialise_monitor_handles();
	printf("Application is running\n");

	uint8_t commandcode;
	uint8_t length;
	GPIO_ButtonInit();

	//Configure the pins
	I2C1_GPIOInits();

	//Initialize the I2C peripheral
	I2C1_Inits();

	//IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV ,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER ,ENABLE);

	//Enable the peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//Enable ACK
	I2C1->CR1 |= (1 << 10);
	while(1)
	{
		//Wait for button press
		while((GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));
		delay();//for de-bouncing

		//Send 0x51 to slave
		commandcode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_NO_SR) != I2C_READY);


		//Read length of data from slave
		while(I2C_MasterReceiveDataIT(&I2C1Handle,&length,1,SLAVE_ADDR,I2C_SR) != I2C_READY);

		//Send 0x52 to slave
		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_SR) != I2C_READY);


		//Read data from slave
		while( I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buff,length,SLAVE_ADDR,I2C_NO_SR) != I2C_READY);
		rxCmplt = RESET;

		//Wait till RX completes
		while(rxCmplt != SET)
		{

		}

		rcv_buff[length+1]='\0';
		printf("Data: %s",rcv_buff);

		rxCmplt = RESET;
	}
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
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		rxCmplt = SET;
	}else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error : Ack failure\n");
		//in master ACK failure happens when slave fails to send ack for the byte from the master
		I2C_CloseSendData(&I2C1Handle);

		//Generate the stop condition
		I2C_GenerateStopCondition(I2C1);

		//hang in infinite loop
		while(1);
	}
}
