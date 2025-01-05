#ifndef ENCODERS_INC_H_
#define ENCODERS_INC_H_

#include "RobotGlobals.h"
#include "stm32f1xx.h"

struct Encoders_Data_Type {
	int Encoder[2];
	};

//------------------------------------------------
struct Encoders_Data_Type *Encoders_GetPointer();

void Encoders_Update(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3);

void Encoders_Init(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3);

void Encoders_SelfTest();

#endif
