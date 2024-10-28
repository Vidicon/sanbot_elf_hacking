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
		GenericArms_HAL_Brake(False, BodyPart);
	}

	if (BodyPart == RightArm)
	{
		RightArm_State.TargetPosition = combined;
		GenericArms_HAL_Brake(False, BodyPart);
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
	// Read limit switches
	int LeftLimitBack = HAL_GPIO_ReadPin(LeftLimitBack_GPIO_Port,LeftLimitBack_Pin);
	int LeftLimitUp = HAL_GPIO_ReadPin(LeftLimitUp_GPIO_Port, LeftLimitUp_Pin);

	int LeftSpeed = 35;
	int RightSpeed = 35;

	//--------------------------------------------------------------------------------
	// Left arm
	//--------------------------------------------------------------------------------
	LeftArm_State.ActualPosition = EncoderData->Encoder[3];

	if (LeftArm_State.HomeState == Arm_NotHomed)
	{
		LeftArm_State.HomeCounter = 0;

		RGBLeds_SetAllColors(LeftArm, Red, LED_On);
	}
	else if (LeftArm_State.HomeState == Arm_Homing)
	{
		GenericArms_HAL_Brake(False, LeftArm);
		LeftArm_State.HomeCounter += 1;

		if (LeftArm_State.HomeCounter <= 1 * UPDATE_20HZ)
		{
			RGBLeds_SetAllColors(LeftArm, Red, LED_Blink_Slow);
			LeftArm_State.Direction = Arm_Motion_MovingUp;
			LeftArm_State.PWM_Output = (100 - abs(20));

			if (LeftLimitUp == 1) { LeftArm_State.HomeCounter = 10 * UPDATE_20HZ;}
		}
		else
		{
			LeftArm_State.Direction = Arm_Motion_MovingDown;
			LeftArm_State.PWM_Output = (100 - abs(20));

			if (LeftLimitBack == 1)
			{
				EncoderData->Encoder[3] = 0;
				Arm_PositionSetpoint(LeftArm, -1, 00);
				LeftArm_State.HomeState = Arm_Homed;

				RGBLeds_SetAllColors(LeftArm, Green, LED_On);
			}
		}
	}
	else if (LeftArm_State.HomeState == Arm_Homed)
	{
		LeftArm_State.ErrorPosition = LeftArm_State.TargetPosition - LeftArm_State.ActualPosition;

		LeftSpeed = abs(LeftArm_State.ErrorPosition/4);

		// Too slow does not work
		if (LeftSpeed < 15) {LeftSpeed = 15;}
		if (LeftSpeed > 50) {LeftSpeed = 50;}

		if (LeftArm_State.ErrorPosition > 15)
		{
			LeftArm_State.Direction = Arm_Motion_MovingDown;
			LeftArm_State.PWM_Output = (100 - abs(LeftSpeed));
			GenericArms_HAL_Brake(False, LeftArm);
		}
		else if (LeftArm_State.ErrorPosition < -15)
		{
			LeftArm_State.Direction = Arm_Motion_MovingUp;
			LeftArm_State.PWM_Output = (100 - abs(LeftSpeed));
			GenericArms_HAL_Brake(False, LeftArm);
		}
		else
		{
			LeftArm_State.PWM_Output = (100 - abs(0));

			// Test if brake works
			GenericArms_HAL_Brake(True, LeftArm);
		}
	}

	//--------------------------------------------------------------------------------
	// Right arm
	//--------------------------------------------------------------------------------
	RightArm_State.ActualPosition = EncoderData->Encoder[4];

	// Read limit switches
	int RightLimitBack = HAL_GPIO_ReadPin(RightLimitBack_GPIO_Port, RightLimitBack_Pin);
	int RightLimitUp = HAL_GPIO_ReadPin(RightLimitUp_GPIO_Port, RightLimitUp_Pin);

	//--------------------------------------------------------------------------------
	// Right arm
	//--------------------------------------------------------------------------------
	RightArm_State.ActualPosition = EncoderData->Encoder[4];

	if (RightArm_State.HomeState == Arm_NotHomed)
	{
		RightArm_State.HomeCounter = 0;

		RGBLeds_SetAllColors(RightArm, Red, LED_On);
	}
	else if (RightArm_State.HomeState == Arm_Homing)
	{
		GenericArms_HAL_Brake(False, RightArm);
		RightArm_State.HomeCounter += 1;

		if (RightArm_State.HomeCounter <= 1 * UPDATE_20HZ)
		{
			RGBLeds_SetAllColors(RightArm, Red, LED_Blink_Slow);
			RightArm_State.Direction = Arm_Motion_MovingDown;
			RightArm_State.PWM_Output = (100 - abs(20));

			if (RightLimitUp == 1) { RightArm_State.HomeCounter = 10 * UPDATE_20HZ;}
		}
		else
		{
			RightArm_State.Direction = Arm_Motion_MovingUp;
			RightArm_State.PWM_Output = (100 - abs(20));

			if (RightLimitBack == 1)
			{
				EncoderData->Encoder[4] = 0;
				Arm_PositionSetpoint(RightArm, 1, 00);
				RightArm_State.HomeState = Arm_Homed;

				RGBLeds_SetAllColors(RightArm, Green, LED_On);
			}
		}
	}
	else if (RightArm_State.HomeState == Arm_Homed)
	{
		RightArm_State.ErrorPosition = RightArm_State.TargetPosition - RightArm_State.ActualPosition;

		RightSpeed = abs(RightArm_State.ErrorPosition/4);

		if (RightSpeed < 15) {RightSpeed = 15;}
		if (RightSpeed > 50) {RightSpeed = 50;}

		if (RightArm_State.ErrorPosition > 15)
		{
			RightArm_State.Direction = Arm_Motion_MovingDown;
			RightArm_State.PWM_Output = (100 - abs(35));
			GenericArms_HAL_Brake(False, RightArm);
		}
		else if (RightArm_State.ErrorPosition < -15)
		{
			RightArm_State.Direction = Arm_Motion_MovingUp;
			RightArm_State.PWM_Output = (100 - abs(35));
			GenericArms_HAL_Brake(False, RightArm);
		}
		else
		{
			RightArm_State.PWM_Output = (100 - abs(0));
			GenericArms_HAL_Brake(True, RightArm);
		}
	}

//	GenericArms_HAL_Brake(False, LeftArm);
//	GenericArms_HAL_Brake(False, RightArm);

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
	LeftArm_State.HomeState = Arm_NotHomed;

	LeftArm_State.TIM = htim;
	LeftArm_State.TIM_CHANNEL = TIM_CHANNEL_1;

	LeftArm_State.TargetPosition = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void LeftArm_Home()
{
	// Move forward a litte
	LeftArm_State.HomeState = Arm_Homing;
	LeftArm_State.HomeCounter = 0;
}

void LeftArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim)
{
	RightArm_State.ArmDirection = Arm_Up;
	RightArm_State.MotionState = Arm_Motion_Disabled;
	RightArm_State.HomeState = Arm_NotHomed;

	RightArm_State.TIM = htim;
	RightArm_State.TIM_CHANNEL = TIM_CHANNEL_2;

	RightArm_State.TargetPosition = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void RightArm_Home()
{
	// Move forward a litte
	RightArm_State.HomeState = Arm_Homing;
	RightArm_State.HomeCounter = 0;
}

void RightArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}
