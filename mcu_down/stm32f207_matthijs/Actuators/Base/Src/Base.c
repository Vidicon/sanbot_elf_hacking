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

char TextBuffer[100];

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

//void Base_VelocitySetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
//{
//	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;
//
//	if (BodyPart == CenterBaseMotor)
//	{
//	}
//
//}

void Base_VelocitySetpoint(int Vx, int Vy, int PhiDot)
{
	// TODO: mapping from XYPhi to 3 motors. Some matrix is needed here.
	CenterBaseMotor_State.SetpointVelocity = PhiDot;
//	CenterBaseMotor_State.MainState = 4;

	if (PhiDot > 0)
	{
		CenterBaseMotor_State.MainState = 2;
	}

	if (PhiDot < 0)
	{
		CenterBaseMotor_State.MainState = 3;
	}

	GenericBase_HAL_Brake(False, LeftBaseMotor);
	GenericBase_HAL_Brake(False, CenterBaseMotor);
	GenericBase_HAL_Brake(False, RightBaseMotor);
}

void CenterBaseMotor_Update20Hz(struct Encoders_Data_Type *EncoderData)
{
	CenterBaseMotor_State.ActualPosition = EncoderData->Encoder[1];
	CenterBaseMotor_State.NewData = EncoderData->NewData[1];

	EncoderData->NewData[1] = 0;

	GenericBase_Update20Hz(&CenterBaseMotor_State, CenterBaseMotor);
}

//void LeftBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData)
//{
//	LeftBaseMotor_State.ActualPosition = EncoderData.Encoder[0];
//	GenericBase_Update20Hz(&LeftBaseMotor_State, LeftBaseMotor);
//}
//
//void RightBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData)
//{
//	RightBaseMotor_State.ActualPosition = EncoderData.Encoder[2];
//	GenericBase_Update20Hz(&RightBaseMotor_State, RightBaseMotor);
//}


void GenericBase_Update20Hz(struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart)
{
	//----------------------------------------------------------------------------
	// Velocity feedback controller
	//----------------------------------------------------------------------------

	Base_State->SetpointPosition += Base_State->SetpointVelocity;
	Base_State->ErrorPosition = Base_State->SetpointPosition - Base_State->ActualPosition;


	//----------------------------------------------------------------------------
	//
	//----------------------------------------------------------------------------
	if (Base_State->NewData == 1)
	{
		Base_State->Differential = Base_State->ErrorPosition - Base_State->ErrorPositionPrev;
	}


	//	Base_State->Integral += Base_State->ErrorPosition;

	int kp = 25 * Base_State->ErrorPosition;
	int Max_Kp = 1000;

	if (kp > Max_Kp) {kp = Max_Kp;}
	if (kp < -Max_Kp) {kp = -Max_Kp;}

	int kd = 25 * Base_State->Differential;
	int Max_Kd = 1000;

	if (kd > Max_Kd) {kd = Max_Kd;}
	if (kd < -Max_Kd) {kd = -Max_Kd;}

	int FF = 400;
//	if (Base_State->MainState == 2 ) { FF = +1;}
//	if (Base_State->MainState == 3 ) { FF = -1;}

//	Base_State->Output = kp + kd + FF * 200 * 0;
	Base_State->Output = kp + kd;

	if (Base_State->Output > 0) {Base_State->Output += FF;}
	if (Base_State->Output > 0) {Base_State->Output -= FF;}



	// Feedback controller
	if (Base_State->Output >= 0)
	{
		Base_State->AmplifierSetpoint = Base_State->Output/100;
		Base_State->MotionState = Base_Motion_MovingDown;
	}
	else
	{
		Base_State->AmplifierSetpoint = -1 * Base_State->Output/100;
		Base_State->MotionState = Base_Motion_MovingUp;
	}


	// At setpoint
	if (Base_State->MainState == 4)
	{
		Base_State->AmplifierSetpoint = 0;
		Base_State->Output = 0;
		Base_State->Integral = 0;
		Base_State->MotionState = Base_Motion_AtTarget;
	}


	//----------------------------------------------------------------------------
	// Write correct outputs
	//----------------------------------------------------------------------------
	if (Base_State->MainState == 0)
	{

	}
	else if (Base_State->MainState == 1)	// Brake

	{

	}
	else if ((Base_State->MainState >= 2) && (Base_State->MainState <= 4))
	{
		// TIM 9 counter = 100.
		int max = 25;
		if (Base_State->AmplifierSetpoint > max)  {Base_State->AmplifierSetpoint = max;}
		if (Base_State->AmplifierSetpoint < -max) {Base_State->AmplifierSetpoint = -max;}

		if (Base_State->MotionState == Base_Motion_MovingUp)
		{
			__HAL_TIM_SET_COMPARE(Base_State->TIM, Base_State->TIM_CHANNEL, Base_State->AmplifierSetpoint);
			GenericBase_HAL_Direction(True, BodyPart);
		}
		else if (Base_State->MotionState == Base_Motion_MovingDown)
		{
			__HAL_TIM_SET_COMPARE(Base_State->TIM, Base_State->TIM_CHANNEL, Base_State->AmplifierSetpoint);
			GenericBase_HAL_Direction(False, BodyPart);
		}
		else if (Base_State->MotionState == Base_Motion_AtTarget)
		{
			__HAL_TIM_SET_COMPARE(Base_State->TIM, Base_State->TIM_CHANNEL, 0);
		}
	}

	// Memory stuff
	Base_State->ErrorPositionPrev = Base_State->ErrorPosition;
	Base_State->ActualVelocity_Prev = Base_State->ActualVelocity;
	Base_State->ActualPosition_Prev =  Base_State->ActualPosition;
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

void GenericBase_HAL_Direction(enum ENUM_Booleans Left, enum ENUM_BodyParts BodyPart)
{
	if ((BodyPart == CenterBaseMotor) && (Left == True))  { HAL_GPIO_WritePin(CenterDir_GPIO_Port, CenterDir_Pin, GPIO_PIN_RESET); }
	if ((BodyPart == CenterBaseMotor) && (Left == False)) { HAL_GPIO_WritePin(CenterDir_GPIO_Port, CenterDir_Pin, GPIO_PIN_SET); }
}


void Temp(int speed)
{
	__HAL_TIM_SET_COMPARE(CenterBaseMotor_State.TIM, CenterBaseMotor_State.TIM_CHANNEL, speed);

//	GenericArm_HAL_Direction(True, BodyPart);
//	HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_SET);

//	GenericArm_HAL_Brake(False, BodyPart);

//	HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port,  LeftArmBrake_Pin,  !BrakeEnable);
}

void TracingUpdate()
{
	memset(TextBuffer, 0x00, 100);

	sprintf(TextBuffer, "%ld,%d,%ld,%ld\n", (long)(CenterBaseMotor_State.SetpointPosition),
											(long)(CenterBaseMotor_State.ActualPosition),
											(long)(CenterBaseMotor_State.Output),
											(long)(CenterBaseMotor_State.MainState ));
	CDC_Transmit_FS((uint8_t*)TextBuffer, strlen(TextBuffer));
}

