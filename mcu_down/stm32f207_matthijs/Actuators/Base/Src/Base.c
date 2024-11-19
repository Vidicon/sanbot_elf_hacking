#include "Base.h"
#include "stm32f2xx.h"
#include <main.h>
#include "Encoders.h"
#include "RGBLeds.h"
#include <stdlib.h>  // For abs()

#include <string.h>		// For tracing
#include <stdio.h>
#include "usbd_cdc_if.h"

struct Base_State_Type LeftBaseMotor_State;
struct Base_State_Type RightBaseMotor_State;
struct Base_State_Type CenterBaseMotor_State;

//char TextBuffer[100];

void Base_Init(TIM_HandleTypeDef *htim9, TIM_HandleTypeDef *htim11, TIM_HandleTypeDef *htim12)
{
	//------------------------------------------------------------------
	LeftBaseMotor_State.TIM = htim12;
	LeftBaseMotor_State.TIM_CHANNEL = TIM_CHANNEL_2;

	LeftBaseMotor_State.MotionState = Base_Motion_Disabled;
	LeftBaseMotor_State.MainState = 0;

	HAL_TIM_Base_Start(htim12);
	HAL_TIM_PWM_Start(htim12, TIM_CHANNEL_2);

	//------------------------------------------------------------------
	CenterBaseMotor_State.TIM = htim11;
	CenterBaseMotor_State.TIM_CHANNEL = TIM_CHANNEL_1;

	CenterBaseMotor_State.MotionState = Base_Motion_Disabled;
	CenterBaseMotor_State.MainState = 0;

	HAL_TIM_Base_Start(htim11);
	HAL_TIM_PWM_Start(htim11, TIM_CHANNEL_1);

	//------------------------------------------------------------------
	RightBaseMotor_State.TIM = htim12;
	RightBaseMotor_State.TIM_CHANNEL = TIM_CHANNEL_1;

	RightBaseMotor_State.MotionState = Base_Motion_Disabled;
	RightBaseMotor_State.MainState = 0;

	HAL_TIM_Base_Start(htim12);
	HAL_TIM_PWM_Start(htim12, TIM_CHANNEL_1);
}

void Base_VelocitySetpoint(int Vx, int Vy, int PhiDot)
{
	//-------------------------------------------------------------------------------------------------
	// http://modwg.co.uk/wp-content/uploads/2015/06/OmniRoller-Holonomic-Drive-Tutorial.pdf
	// M1 = right base motor
	// M2 = left base motor
	// M3 = center base motor
	// -100 --> 0 --> +100
	// X = sideways (for Sara robot "to the right")
	// Y = forward
	// Phi = rotate Rz.
	// Vx = velocity in X
	// Vy = velocity in Y
	// PhiDot = rotational velocity around Rz.
	//-------------------------------------------------------------------------------------------------

	long M1 = 0;
	long M2 = 0;
	long M3 = 0;

	// Scaling = 100 for speed & 100 for gains.
	// M_inv = 1/3 * ((-1, sqrt(3), 1),(-1, -sqrt(3), 1), (2 0 1))

	M1 = -100 * Vx + 173 * Vy + 100 * PhiDot;
	M2 = -100 * Vx - 173 * Vy + 100 * PhiDot;
	M3 = +200 * Vx +   0 * Vy + 100 * PhiDot;

	M1 = M1 / 200;
	M2 = M2 / 200;
	M3 = M3 / 200;

	if (M1 > 0) { RightBaseMotor_State.Direction = Base_Motion_Negative;} else {RightBaseMotor_State.Direction = Base_Motion_Positive;}
	RightBaseMotor_State.PWM_Output = (100 - abs(M1));

	if (M2 > 0) { LeftBaseMotor_State.Direction = Base_Motion_Negative;} else {LeftBaseMotor_State.Direction = Base_Motion_Positive;}
	LeftBaseMotor_State.PWM_Output = (100 - abs(M2));

	if (M3 > 0) { CenterBaseMotor_State.Direction = Base_Motion_Negative;} else {CenterBaseMotor_State.Direction = Base_Motion_Positive;}
	CenterBaseMotor_State.PWM_Output = (100 - abs(M3));

	GenericBase_HAL_Brake(False, LeftBaseMotor);
	GenericBase_HAL_Brake(False, CenterBaseMotor);
	GenericBase_HAL_Brake(False, RightBaseMotor);
}

void Base_Update20Hz(struct Encoders_Data_Type *EncoderData)
{
	LeftBaseMotor_State.ActualPosition = EncoderData->Encoder[0];
	CenterBaseMotor_State.ActualPosition = EncoderData->Encoder[1];
	RightBaseMotor_State.ActualPosition = EncoderData->Encoder[2];

	GenericBase_HAL_Brake(False, LeftBaseMotor);
	GenericBase_HAL_Brake(False, CenterBaseMotor);
	GenericBase_HAL_Brake(False, RightBaseMotor);

	GenericBase_HAL_Direction(LeftBaseMotor_State.Direction, LeftBaseMotor);
	GenericBase_HAL_Direction(CenterBaseMotor_State.Direction, CenterBaseMotor);
	GenericBase_HAL_Direction(RightBaseMotor_State.Direction, RightBaseMotor);

	GenericBase_HAL_PWM(LeftBaseMotor_State.PWM_Output, LeftBaseMotor);
	GenericBase_HAL_PWM(CenterBaseMotor_State.PWM_Output, CenterBaseMotor);
	GenericBase_HAL_PWM(RightBaseMotor_State.PWM_Output, RightBaseMotor);
}

void GenericBase_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart)
{
	//	 Works inverted. High to release brake.
	if (BodyPart == LeftBaseMotor)		{ HAL_GPIO_WritePin(LeftBrake_GPIO_Port, LeftBrake_Pin, !BrakeEnable); }
	if (BodyPart == CenterBaseMotor)	{ HAL_GPIO_WritePin(CenterBrake_GPIO_Port, CenterBrake_Pin, !BrakeEnable); }
	if (BodyPart == RightBaseMotor)		{ HAL_GPIO_WritePin(RightBrake_GPIO_Port, RightBrake_Pin, !BrakeEnable); }
}

void GenericBase_HAL_Direction(enum ENUM_BaseMotionState Direction, enum ENUM_BodyParts BodyPart)
{
	if ((BodyPart == LeftBaseMotor) && (Direction == Base_Motion_Positive))  { HAL_GPIO_WritePin(LeftDir_GPIO_Port, LeftDir_Pin, GPIO_PIN_RESET); }
	if ((BodyPart == LeftBaseMotor) && (Direction == Base_Motion_Negative)) { HAL_GPIO_WritePin(LeftDir_GPIO_Port, LeftDir_Pin, GPIO_PIN_SET); }

	if ((BodyPart == CenterBaseMotor) && (Direction == Base_Motion_Positive))  { HAL_GPIO_WritePin(CenterDir_GPIO_Port, CenterDir_Pin, GPIO_PIN_RESET); }
	if ((BodyPart == CenterBaseMotor) && (Direction == Base_Motion_Negative))  { HAL_GPIO_WritePin(CenterDir_GPIO_Port, CenterDir_Pin, GPIO_PIN_SET); }

	if ((BodyPart == RightBaseMotor) && (Direction == Base_Motion_Positive))  { HAL_GPIO_WritePin(RightDir_GPIO_Port, RightDir_Pin, GPIO_PIN_RESET); }
	if ((BodyPart == RightBaseMotor) && (Direction == Base_Motion_Negative)) { HAL_GPIO_WritePin(RightDir_GPIO_Port, RightDir_Pin, GPIO_PIN_SET); }
}


void GenericBase_HAL_PWM(int PWM, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftBaseMotor)
	{
		__HAL_TIM_SET_COMPARE(LeftBaseMotor_State.TIM, LeftBaseMotor_State.TIM_CHANNEL, PWM);
	}

	if (BodyPart == CenterBaseMotor)
	{
		__HAL_TIM_SET_COMPARE(CenterBaseMotor_State.TIM, CenterBaseMotor_State.TIM_CHANNEL, PWM);
	}

	if (BodyPart == RightBaseMotor)
	{
		__HAL_TIM_SET_COMPARE(RightBaseMotor_State.TIM, RightBaseMotor_State.TIM_CHANNEL, PWM);
	}
}

//void TracingUpdate()
//{
//	memset(TextBuffer, 0x00, 100);
//
//	sprintf(TextBuffer, "%ld,%d,%ld\n",
//												(long)(LeftBaseMotor_State.ActualPosition),
//												(long)(CenterBaseMotor_State.ActualPosition),
//												(long)(RightBaseMotor_State.ActualPosition));
//	CDC_Transmit_FS((uint8_t*)TextBuffer, strlen(TextBuffer));
//}

