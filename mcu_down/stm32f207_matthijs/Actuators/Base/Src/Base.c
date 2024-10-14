#include "Base.h"
#include "stm32f2xx.h"
#include <main.h>
#include "Encoders.h"
#include "RGBLeds.h"
#include <stdlib.h>  // For abs()

struct Base_State_Type LeftBaseMotor_State;
struct Base_State_Type RightBaseMotor_State;
struct Base_State_Type CenterBaseMotor_State;

void Base_Init(TIM_HandleTypeDef *htim9, TIM_HandleTypeDef *htim11)
{
//	LeftMotor_State.TIM = htim;
//	LeftMotor_State.TIM_CHANNEL = TIM_CHANNEL_3;
//
//	LeftMotor_State.MotionState = Base_Motion_Disabled;
//	LeftMotor_State.MainState = 0;
//
//	RightMotor_State.TIM = htim;
//	RightMotor_State.TIM_CHANNEL = TIM_CHANNEL_4;
//
//	RightMotor_State.MotionState = Base_Motion_Disabled;
//	RightMotor_State.MainState = 0;

	CenterBaseMotor_State.TIM = htim11;
	CenterBaseMotor_State.TIM_CHANNEL = TIM_CHANNEL_1;

	CenterBaseMotor_State.MotionState = Base_Motion_Disabled;
	CenterBaseMotor_State.MainState = 0;

	HAL_TIM_Base_Start(htim11);
	HAL_TIM_PWM_Start(htim11, TIM_CHANNEL_1);

//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_5);
}

void Base_VelocitySetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
{
	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;

	if (BodyPart == CenterBaseMotor)
	{
	}
}


void CenterBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData)
{
	CenterBaseMotor_State.ActualPosition = EncoderData.Encoder[1];
	GenericBase_Update20Hz(EncoderData, &CenterBaseMotor_State, CenterBaseMotor);
}

void LeftBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData)
{
	LeftBaseMotor_State.ActualPosition = EncoderData.Encoder[0];
	GenericBase_Update20Hz(EncoderData, &LeftBaseMotor_State, LeftBaseMotor);
}

void RightBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData)
{
	RightBaseMotor_State.ActualPosition = EncoderData.Encoder[0];
	GenericBase_Update20Hz(EncoderData, &RightBaseMotor_State, RightBaseMotor);
}


void GenericBase_Update20Hz(struct Encoders_Data_Type EncoderData, struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart)
{

}

void GenericBase_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart)
{
	//	 Works inverted. High to release brake.
	if (BodyPart == LeftBaseMotor)
	{
		HAL_GPIO_WritePin(LeftBrake_GPIO_Port, LeftBrake_Pin, !BrakeEnable);
	}

	if (BodyPart == CenterBaseMotor)
	{
		HAL_GPIO_WritePin(CenterBrake_GPIO_Port, CenterBrake_Pin, !BrakeEnable);
	}

	if (BodyPart == RightBaseMotor)
	{
		HAL_GPIO_WritePin(RightBrake_GPIO_Port, RightBrake_Pin, !BrakeEnable);
	}
}

void Temp(int speed)
{
	__HAL_TIM_SET_COMPARE(CenterBaseMotor_State.TIM, CenterBaseMotor_State.TIM_CHANNEL, speed);

//	GenericArm_HAL_Direction(True, BodyPart);
//	HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_SET);

//	GenericArm_HAL_Brake(False, BodyPart);

//	HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port,  LeftArmBrake_Pin,  !BrakeEnable);
}
