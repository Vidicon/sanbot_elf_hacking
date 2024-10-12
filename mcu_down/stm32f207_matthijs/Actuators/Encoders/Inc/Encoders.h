/*
 * Arms.h
 *
 *  Created on: May 3, 2024
 *      Author: matthijs
 */

#ifndef ENCODERS_INC_H_
#define ENCODERS_INC_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"
#include <stm32f2xx_hal_uart.h>

struct Encoders_Data_Type {
	int Encoder[5];
	};


//------------------------------------------------
struct Encoders_Data_Type Encoders_GetPointer();

void Encoders_Init(UART_HandleTypeDef *huart);

void Encoders_SelfTest();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
