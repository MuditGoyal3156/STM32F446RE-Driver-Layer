/*
 * stm32f446xx_I2C_driver.h
 *
 *  Created on: Dec 18, 2025
 *      Author: mudit
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t ;

typedef struct
{
	I2C_RegDef_t *pI2Cx;  //This holds the base address of the I2Cx peripheral
	I2C_Config_t I2C_Config;
	uint8_t	*pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint8_t RxSize;
	uint8_t Sr;

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_SM2K	200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


#define I2C_NO_SR		RESET
#define I2C_SR			SET

/*
 * I2C Application State
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 * I2c Application macros
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO 		4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
