/*
 * RCC_Peripheral.h
 *
 *  Created on: Dec 25, 2025
 *      Author: mudit
 */

#ifndef INC_RCC_PERIPHERAL_H_
#define INC_RCC_PERIPHERAL_H_
#include "stm32f446xx.h"
#include <stdint.h>

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_RCC_PERIPHERAL_H_ */
