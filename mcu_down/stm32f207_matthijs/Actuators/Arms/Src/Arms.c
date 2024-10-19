#include "Arms.h"
#include "stm32f2xx.h"
#include <main.h>
#include "Encoders.h"
#include "RGBLeds.h"
#include <stdlib.h>  // For abs()

struct Arm_State_Type LeftArm_State;
struct Arm_State_Type RightArm_State;


void Arm_PositionSetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
{
	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;

	if (BodyPart == LeftArm)
	{
		LeftArm_State.TargetPosition = combined;
	}

	if (BodyPart == RightArm)
	{
		RightArm_State.TargetPosition = combined;
	}
}

void GenericArms_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart)
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

void GenericArms_HAL_Direction(enum ENUM_ArmMotionState Direction, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftArm)
	{
		if (Direction == Arm_Motion_MovingUp)
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
		if (Direction == Arm_Motion_MovingUp)
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_RESET);
		}
	}
}

void GenericArms_HAL_PWM(int PWM, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftArm)
	{
		__HAL_TIM_SET_COMPARE(LeftArm_State.TIM, LeftArm_State.TIM_CHANNEL, PWM);
	}

	if (BodyPart == RightArm)
	{
		__HAL_TIM_SET_COMPARE(RightArm_State.TIM, RightArm_State.TIM_CHANNEL, PWM);
	}
}

void Arms_Update20Hz(struct Encoders_Data_Type *EncoderData)
{
	//--------------------------------------------------------------------------------
	// Left arm
	//--------------------------------------------------------------------------------
	LeftArm_State.ActualPosition = EncoderData->Encoder[3];
	LeftArm_State.ErrorPosition = LeftArm_State.TargetPosition - LeftArm_State.ActualPosition;

	if (LeftArm_State.ErrorPosition > 20)
	{
		LeftArm_State.Direction = Arm_Motion_MovingDown;
		LeftArm_State.PWM_Output = (100 - abs(35));
	}
	else if (LeftArm_State.ErrorPosition > 10)
	{
		LeftArm_State.Direction = Arm_Motion_MovingDown;
		LeftArm_State.PWM_Output = (100 - abs(20));
	}
	else if (LeftArm_State.ErrorPosition < -20)
	{
		LeftArm_State.Direction = Arm_Motion_MovingUp;
		LeftArm_State.PWM_Output = (100 - abs(35));
	}
	else if (LeftArm_State.ErrorPosition < -10)
	{
		LeftArm_State.Direction = Arm_Motion_MovingUp;
		LeftArm_State.PWM_Output = (100 - abs(20));
	}
	else
	{
		LeftArm_State.PWM_Output = (100 - abs(0));
	}

	//--------------------------------------------------------------------------------
	// Right arm
	//--------------------------------------------------------------------------------
	RightArm_State.ActualPosition = EncoderData->Encoder[4];
	RightArm_State.ErrorPosition = RightArm_State.TargetPosition - RightArm_State.ActualPosition;

	if (RightArm_State.ErrorPosition > 20)
	{
		RightArm_State.Direction = Arm_Motion_MovingDown;
		RightArm_State.PWM_Output = (100 - abs(35));
	}
	else if (RightArm_State.ErrorPosition > 10)
	{
		RightArm_State.Direction = Arm_Motion_MovingDown;
		RightArm_State.PWM_Output = (100 - abs(20));
	}
	else if (RightArm_State.ErrorPosition < -20)
	{
		RightArm_State.Direction = Arm_Motion_MovingUp;
		RightArm_State.PWM_Output = (100 - abs(35));
	}
	else if (RightArm_State.ErrorPosition < -10)
	{
		RightArm_State.Direction = Arm_Motion_MovingUp;
		RightArm_State.PWM_Output = (100 - abs(20));
	}
	else
	{
		RightArm_State.PWM_Output = (100 - abs(0));
	}



	GenericArms_HAL_Brake(False, LeftArm);
	GenericArms_HAL_Brake(False, RightArm);

	GenericArms_HAL_Direction(LeftArm_State.Direction, LeftArm);
	GenericArms_HAL_Direction(RightArm_State.Direction, RightArm);

	GenericArms_HAL_PWM(LeftArm_State.PWM_Output, LeftArm);
	GenericArms_HAL_PWM(RightArm_State.PWM_Output, RightArm);
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim)
{
	LeftArm_State.ArmDirection = Arm_Up;
	LeftArm_State.MotionState = Arm_Motion_Disabled;

	LeftArm_State.TIM = htim;
	LeftArm_State.TIM_CHANNEL = TIM_CHANNEL_1;

	LeftArm_State.TargetPosition = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);

}

void LeftArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}

//void LeftArm_Update20Hz(struct Encoders_Data_Type *EncoderData)
//{
//	LeftArm_State.ActualPosition = EncoderData->Encoder[3];
//	GenericArm_Update20Hz(&LeftArm_State, LeftArm);
//}
//
//void LeftArm_HAL_Brake(enum ENUM_Booleans BrakeEnable)
//{
////	 Works inverted. High to release brake.
//	HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port, LeftArmBrake_Pin, !BrakeEnable);
//}

//void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
//{
//	// Request state 1 to enable the brake.
//	// Disable any controller running.
//	LeftArm_State.MainState = 1;
//}
//
//void LeftArm_NewSetpoint(int NewSetpoint)
//{
//	LeftArm_State.VelocitySetpoint =
//
//}Arm
//
//void Command_NewVelocitySetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
//{
//	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;
//
//	if (BodyPart == LeftArm)
//	{
//		LeftArm_NewSetpoint(combined);
//	}
//
//	if (BodyPart == RightArm)
//	{
//		RightArm_NewSetpoint(combined);
//	}
//}
//
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

//void RightArm_SelfTest(enum ENUM_Booleans Enabled)
//{
//	// To do
//}
//
//void RightArm_Update20Hz(struct Encoders_Data_Type *EncoderData)
//{
//	RightArm_State.ActualPosition = EncoderData->Encoder[4];
//	GenericArm_Update20Hz(&RightArm_State, RightArm);
//}
//
//void RightArm_HAL_Brake(enum ENUM_Booleans BrakeEnable)
//{
////	 Works inverted. High to release brake.
//	HAL_GPIO_WritePin(RightArmBrake_GPIO_Port, RightArmBrake_Pin, !BrakeEnable);
//}
//
//void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
//{
//	// Request state 1 to enable the brake.
//	// Disable any controller running.
//	RightArm_State.MainState = 1;
//}
//
//void RightArm_NewSetpoint(int NewSetpoint)
//{
//	// Check if new setpoint is different from current position
//	RightArm_State.TargetPosition = NewSetpoint;
//
//	RightArm_State.MainState = 0;
//
//	if (NewSetpoint > RightArm_State.ActualPosition)
//	{
//		RightArm_State.MainState  = 2;
//	}
//
//	if (NewSetpoint < RightArm_State.ActualPosition)
//	{
//		RightArm_State.MainState  = 3;
//	}
//
//	RightArm_State.BrakeWindow = 10 + abs(RightArm_State.ActualPosition - NewSetpoint) / 10;
//}
