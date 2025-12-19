/*
 * 007_spi_txonly_arduino.c
 *
 *  Created on: Dec 13, 2025
 *      Author: mudit
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>

//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//arduino led pin
#define LED_PIN			9

void delay(void)
{
	for(uint32_t i = 0 ; i< 250000 ; i++);
}

/*
 * Alternate Function Mode : 5
 * PB12 --> NSS
 * PB13 --> SCK
 * PB14 --> MISO
 * PB15 --> MOSI
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//Hardware slave management

	SPI_Init(&SPI2handle);
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

uint8_t SPI_verifyresponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}
int main(void)
{
	uint8_t dummy = 0xff;//8 bit data transfer
	uint8_t dummy_read;

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2 , ENABLE);
	GPIO_ButtonInit();

	while(1)
	{
		while((GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));

		delay();

		//Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2 ,ENABLE);

		//1. CMD_LED_CTRL	<pin no(1)>		<value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;

		uint8_t ackbyte;

		uint8_t arg[2];
		//First send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//send some dummy bits(1byte) to fetch the response form the slave
		SPI_SendData(SPI2,&dummy,1);

		//Receive ACK or NACK from slave
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if(SPI_verifyresponse(ackbyte))
		{
			//send argument
			arg[0] = LED_PIN;
			arg[1] = LED_ON;
			SPI_SendData(SPI2,arg,2);
		}


		//2. CMD_SENSOR_READ	<analog pin number(1)>

		while(!(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));
		delay();
		commandcode = COMMAND_SENSOR_READ;
		//First send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//send some dummy bits(1byte) to fetch the response form the slave
		SPI_SendData(SPI2,&dummy,1);

		//Receive ACK or NACK from slave
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if(SPI_verifyresponse(ackbyte))
		{
			//send argument
			arg[0] = ANALOG_PIN0;
			SPI_SendData(SPI2,arg,1);

			//Read sensor value so that slave can be ready with data

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay
			delay();

			//send some dummy bits(1byte) to fetch the response form the slave
			SPI_SendData(SPI2,&dummy,1);

			uint8_t analog_read;

			SPI_ReceiveData(SPI2,&analog_read,1);
		}
		//3.  CMD_LED_READ 	 <pin no(1) >


		while(!(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));

		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_verifyresponse(ackbyte))
		{
			arg[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,arg,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		while(!(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));

		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_verifyresponse(ackbyte))
		{
			arg[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,arg,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < arg[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}
		}

		//5. CMD_ID_READ
		while(!(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));
		delay();

		commandcode = COMMAND_ID_READ;
		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_verifyresponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';
		}


		while ((SPI2->SR & (1 << 7)));
		//Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2 ,DISABLE);


	}
}
