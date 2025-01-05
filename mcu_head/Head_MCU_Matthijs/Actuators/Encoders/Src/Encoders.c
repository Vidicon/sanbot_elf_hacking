#include "Encoders.h"
#include "stm32f1xx.h"
#include <main.h>
#include <stdio.h>
#include <string.h>

struct Encoders_Data_Type EncoderData;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct Encoders_Data_Type *Encoders_GetPointer()
{
	return &EncoderData;
}

void Encoders_Update()
{
	EncoderData.Encoder[0] = __HAL_TIM_GET_COUNTER(EncoderData.TIM[0]);
	EncoderData.Encoder[1] = __HAL_TIM_GET_COUNTER(EncoderData.TIM[1]);
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void Encoders_Init(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3)
{
	EncoderData.TIM[0] = htim1;
	EncoderData.TIM[1] = htim3;

	__HAL_TIM_SET_COUNTER(htim1, 0);
	__HAL_TIM_SET_COUNTER(htim3, 0);

	HAL_TIM_Encoder_Start(htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	EncoderData.Encoder[0] = 0;
	EncoderData.Encoder[1] = 0;
}

void Encoders_SelfTest()
{

}


