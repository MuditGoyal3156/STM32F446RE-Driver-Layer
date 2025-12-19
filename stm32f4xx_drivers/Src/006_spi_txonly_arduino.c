/*
 * 007_spi_txonly_arduino.c
 *
 *  Created on: Dec 13, 2025
 *      Author: mudit
 */




#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>

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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV64;
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
int main(void)
{
	char user_data[]="Hello world";

	SPI2_GPIOInits();

	SPI2_Inits();

	GPIO_ButtonInit();

	SPI_SSOEConfig(SPI2 , ENABLE);


	while(1)
	{
	while((GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)));

	delay();

	//Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2 ,ENABLE);

	//First send length information
	uint8_t datalen = strlen(user_data);
	SPI_SendData(SPI2,&datalen,1);

	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	while ((SPI2->SR & (1 << 7)));
	//Disable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2 ,DISABLE);

	}
}
