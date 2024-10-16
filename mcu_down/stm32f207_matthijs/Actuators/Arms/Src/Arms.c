#include "Arms.h"
#include "stm32f2xx.h"
#include <main.h>
#include "Encoders.h"
#include "RGBLeds.h"
#include <stdlib.h>  // For abs()

struct Arm_State_Type LeftArm_State;
struct Arm_State_Type RightArm_State;


void Command_NewSetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
{
	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;

	if (BodyPart == LeftArm)
	{
		LeftArm_NewSetpoint(combined);
	}

	if (BodyPart == RightArm)
	{
		RightArm_NewSetpoint(combined);
	}
}

void GenericArm_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart)
{
	//	 Works inverted. High to release brake.
	if (BodyPart == LeftArm)
	{
		HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port,  LeftArmBrake_Pin,  !BrakeEnable);
	}

	if (BodyPart == RightArm)
	{
		HAL_GPIO_WritePin(RightArmBrake_GPIO_Port, RightArmBrake_Pin, !BrakeEnable);
	}
}

void GenericArm_HAL_Direction(enum ENUM_Booleans Up, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftArm)
	{
		if (Up == True)
		{
			HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_RESET);
		}
	}

	if (BodyPart == RightArm)
	{
		if (Up == True)
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_RESET);
		}
	}
}

void GenericArm_Update20Hz(struct Encoders_Data_Type EncoderData, struct Arm_State_Type *Arm_State, enum ENUM_BodyParts BodyPart)
{
	//	0 = setpoint off, brake off
	//  1 = setpoint off, brake ON
	//	2 = setpoint pos
	//	3 = setpoint neg
	//	4 = setpoint ready,brake ON

	//----------------------------------------------------------------------------
	// Feedback controller
	//----------------------------------------------------------------------------
	if ((Arm_State->MainState >= 2) && (Arm_State->MainState <= 4))
	{
		Arm_State->SetpointPosition = Arm_State->TargetPosition;
		Arm_State->ErrorPosition = Arm_State->SetpointPosition - Arm_State->ActualPosition;

		// Check end-condition
		if ((Arm_State->MainState == 2) && (Arm_State->ErrorPosition <= +Arm_State->BrakeWindow)) { Arm_State->MainState = 4;}
		if ((Arm_State->MainState == 3) && (Arm_State->ErrorPosition >= -Arm_State->BrakeWindow)) { Arm_State->MainState = 4;}


		if (Arm_State->MainState == 2)
		{
			RGBLeds_SetColorOff(BodyPart);
			RGBLeds_SetColorOn(BodyPart, Blue);
		}

		if (Arm_State->MainState == 3)
		{
			RGBLeds_SetColorOff(BodyPart);
			RGBLeds_SetColorOn(BodyPart, Green);
		}

//		if (Arm_State->MainState == 4)
//		{
//			RGBLeds_SetColorOff(BodyPart);
//			RGBLeds_SetColorOn(BodyPart, Green);
//		}

		//----------------------------------------------------------------------------
		//
		//----------------------------------------------------------------------------
		Arm_State->Differential = Arm_State->ErrorPosition - Arm_State->ErrorPositionPrev;
		Arm_State->ErrorPositionPrev = Arm_State->ErrorPosition;
		Arm_State->Integral += Arm_State->ErrorPosition;

		int kp = 40 * Arm_State->ErrorPosition;
		int Max_Kp = 800;

		if (kp > Max_Kp) {kp = Max_Kp;}
		if (kp < -Max_Kp) {kp = -Max_Kp;}

		int kd = 30 * Arm_State->Differential;
		int Max_Kd = 500;

		if (kd > Max_Kd) {kd = Max_Kd;}
		if (kd < -Max_Kd) {kd = -Max_Kd;}

		int FF = 0;
		if (Arm_State->MainState == 2 ) { FF = +1;}
		if (Arm_State->MainState == 3 ) { FF = -1;}

		Arm_State->Output = kp + kd + FF * 1000;

		// Feedback controller
		if (Arm_State->Output >= 0)
		{
			Arm_State->AmplifierSetpoint = Arm_State->Output/100;
			Arm_State->MotionState = Arm_Motion_MovingDown;
		}
		else
		{
			Arm_State->AmplifierSetpoint = -1 * Arm_State->Output/100;
			Arm_State->MotionState = Arm_Motion_MovingUp;
		}
	}

	// Move done
	if (Arm_State->MainState == 4)
	{
		Arm_State->AmplifierSetpoint = 0;
		Arm_State->Output = 0;
		Arm_State->Integral = 0;
		Arm_State->MotionState = Arm_Motion_AtTarget;
	}

	//----------------------------------------------------------------------------
	// Write correct outputs
	//----------------------------------------------------------------------------
	if (Arm_State->MainState == 0)
	{
		__HAL_TIM_SET_COMPARE(Arm_State->TIM, Arm_State->TIM_CHANNEL, 0);
		GenericArm_HAL_Brake(False, BodyPart);
	}
	else if (Arm_State->MainState == 1)
	{
		__HAL_TIM_SET_COMPARE(Arm_State->TIM, Arm_State->TIM_CHANNEL, 0);
		GenericArm_HAL_Brake(True, BodyPart);
	}
	else if ((Arm_State->MainState >= 2) && (Arm_State->MainState <= 4))
	{
		// TIM 9 counter = 100.
		int max = 25;
		if (Arm_State->AmplifierSetpoint > max)  {Arm_State->AmplifierSetpoint = max;}
		if (Arm_State->AmplifierSetpoint < -max) {Arm_State->AmplifierSetpoint = -max;}

		if (Arm_State->MotionState == Arm_Motion_MovingUp)
		{
			__HAL_TIM_SET_COMPARE(Arm_State->TIM, Arm_State->TIM_CHANNEL, Arm_State->AmplifierSetpoint);
			GenericArm_HAL_Direction(True, BodyPart);
			GenericArm_HAL_Brake(False, BodyPart);
		}
		else if (Arm_State->MotionState == Arm_Motion_MovingDown)
		{
			__HAL_TIM_SET_COMPARE(Arm_State->TIM, Arm_State->TIM_CHANNEL, Arm_State->AmplifierSetpoint);
			GenericArm_HAL_Direction(False, BodyPart);
			GenericArm_HAL_Brake(False, BodyPart);
		}
		else if (Arm_State->MotionState == Arm_Motion_AtTarget)
		{
			__HAL_TIM_SET_COMPARE(Arm_State->TIM, Arm_State->TIM_CHANNEL, 0);
			GenericArm_HAL_Brake(True, BodyPart);
		}
	}
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim)
{
	LeftArm_State.ArmDirection = Arm_Up;
	LeftArm_State.MotionState = Arm_Motion_Disabled;
	LeftArm_State.MainState = 0;

	LeftArm_State.TIM = htim;
	LeftArm_State.TIM_CHANNEL = TIM_CHANNEL_1;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void LeftArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}

void LeftArm_Update20Hz(struct Encoders_Data_Type EncoderData)
{
	LeftArm_State.ActualPosition = EncoderData.Encoder[3];
	GenericArm_Update20Hz(EncoderData, &LeftArm_State, LeftArm);
}

void LeftArm_HAL_Brake(enum ENUM_Booleans BrakeEnable)
{
//	 Works inverted. High to release brake.
	HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port, LeftArmBrake_Pin, !BrakeEnable);
}

void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
{
	// Request state 1 to enable the brake.
	// Disable any controller running.
	LeftArm_State.MainState = 1;
}

void LeftArm_NewSetpoint(int NewSetpoint)
{
	// Check if new setpoint is different from current position
	LeftArm_State.TargetPosition = NewSetpoint;

	LeftArm_State.MainState = 0;

	if (NewSetpoint > LeftArm_State.ActualPosition)
	{
		LeftArm_State.MainState  = 2;
	}

	if (NewSetpoint < LeftArm_State.ActualPosition)
	{
		LeftArm_State.MainState  = 3;
	}

	LeftArm_State.BrakeWindow = 10 + abs(LeftArm_State.ActualPosition - NewSetpoint) / 10;
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim)
{
	RightArm_State.ArmDirection = Arm_Up;
	RightArm_State.MotionState = Arm_Motion_Disabled;
	RightArm_State.TIM = htim;

	RightArm_State.TIM = htim;
	RightArm_State.TIM_CHANNEL = TIM_CHANNEL_2;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void RightArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}

void RightArm_Update20Hz(struct Encoders_Data_Type EncoderData)
{
	RightArm_State.ActualPosition = EncoderData.Encoder[4];
	GenericArm_Update20Hz(EncoderData, &RightArm_State, RightArm);
}

void RightArm_HAL_Brake(enum ENUM_Booleans BrakeEnable)
{
//	 Works inverted. High to release brake.
	HAL_GPIO_WritePin(RightArmBrake_GPIO_Port, RightArmBrake_Pin, !BrakeEnable);
}

void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
{
	// Request state 1 to enable the brake.
	// Disable any controller running.
	RightArm_State.MainState = 1;
}

void RightArm_NewSetpoint(int NewSetpoint)
{
	// Check if new setpoint is different from current position
	RightArm_State.TargetPosition = NewSetpoint;

	RightArm_State.MainState = 0;

	if (NewSetpoint > RightArm_State.ActualPosition)
	{
		RightArm_State.MainState  = 2;
	}

	if (NewSetpoint < RightArm_State.ActualPosition)
	{
		RightArm_State.MainState  = 3;
	}

	RightArm_State.BrakeWindow = 10 + abs(RightArm_State.ActualPosition - NewSetpoint) / 10;
}
