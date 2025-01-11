#include "Arms.h"
#include "stm32f2xx.h"
#include <main.h>
#include "Encoders.h"
#include "RGBLeds.h"
#include <stdlib.h>  // For abs()

struct Arm_State_Type LeftArm_State;
struct Arm_State_Type RightArm_State;

void Generic_Arm_PositionSetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
{
	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;

	if (BodyPart == LeftArm)
	{
		LeftArm_State.TargetPosition = combined;
		GenericArms_HAL_Brake(False, BodyPart);
		LeftArm_State.MotionState = Motion_Moving;

	}

	if (BodyPart == RightArm)
	{
		RightArm_State.TargetPosition = combined;
		GenericArms_HAL_Brake(False, BodyPart);
		RightArm_State.MotionState = Motion_Moving;
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

void GenericArms_HAL_Direction(enum ENUM_ArmDirection Direction, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftArm)
	{
		if (Direction == Arm_Up)
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
		if (Direction == Arm_Up)
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_SET);
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
	int LeftSpeed = 35;

	//--------------------------------------------------------------------------------
	// Read limit switches
	//--------------------------------------------------------------------------------
	int LeftLimitBack = HAL_GPIO_ReadPin(LeftLimitBack_GPIO_Port,LeftLimitBack_Pin);
	int LeftLimitUp = HAL_GPIO_ReadPin(LeftLimitUp_GPIO_Port, LeftLimitUp_Pin);

	//--------------------------------------------------------------------------------
	// Invert direction. Make UP = +
	//--------------------------------------------------------------------------------
	LeftArm_State.ActualPosition = -1 * EncoderData->Encoder[3];

	if (LeftArm_State.HomeState < Homed)
	{
		if (LeftArm_State.HomeState == NotHomed)
		{
			LeftArm_State.HomeCounter = 0;

			RGBLeds_SetAllColors(LeftArm, Red, LED_On);
		}
		else if (LeftArm_State.HomeState == Homing)
		{
			GenericArms_HAL_Brake(False, LeftArm);
			LeftArm_State.HomeCounter += 1;

			LeftArm_State.Direction = Arm_Down;
			LeftArm_State.PWM_Output = (100 - abs(20));

			if (LeftLimitBack == 1)
			{
				// Set current encoder position to zero
				EncoderData->Encoder[3] = 0;

				LeftArm_State.PWM_Output = (100 - abs(0));

				Generic_Arm_PositionSetpoint(LeftArm, 1, 00);
				LeftArm_State.HomeState = Homed;

				RGBLeds_SetAllColors(LeftArm, Green, LED_On);
			}
		}
	}
	else if (LeftArm_State.MotionState == Motion_Moving)
	{
		LeftArm_State.ErrorPosition = LeftArm_State.TargetPosition - LeftArm_State.ActualPosition;

		LeftSpeed = abs(LeftArm_State.ErrorPosition/4);

		// Too slow does not work
		if (LeftSpeed < 15) {LeftSpeed = 15;}
		if (LeftSpeed > 50) {LeftSpeed = 50;}

		if (LeftArm_State.ErrorPosition > 15)
		{
			LeftArm_State.Direction = Arm_Up;
			LeftArm_State.PWM_Output = (100 - abs(LeftSpeed));
			GenericArms_HAL_Brake(False, LeftArm);
		}
		else if (LeftArm_State.ErrorPosition < -15)
		{
			LeftArm_State.Direction = Arm_Down;
			LeftArm_State.PWM_Output = (100 - abs(LeftSpeed));
			GenericArms_HAL_Brake(False, LeftArm);
		}
		else
		{
			LeftArm_State.PWM_Output = (100 - abs(0));
			LeftArm_State.MotionState = Motion_Breaking;
			LeftArm_State.BrakeTimer = 0;
		}
	}
	else if (LeftArm_State.MotionState == Motion_Breaking)
	{
		LeftArm_State.PWM_Output = (100 - abs(0));
		GenericArms_HAL_Brake(True, LeftArm);

		LeftArm_State.BrakeTimer += 1;

		if (LeftArm_State.BrakeTimer >= 1 * UPDATE_20HZ)
		{
			LeftArm_State.MotionState = Motion_Idle;
		}
	}
	else if (LeftArm_State.MotionState == Motion_Idle)
	{
		LeftArm_State.PWM_Output = (100 - abs(0));
		GenericArms_HAL_Brake(False, LeftArm);
	}

	//--------------------------------------------------------------------------------
	// Right arm
	//--------------------------------------------------------------------------------
	int RightSpeed = 35;

	//--------------------------------------------------------------------------------
	// Read limit switches
	//--------------------------------------------------------------------------------
	int RightLimitBack = HAL_GPIO_ReadPin(RightLimitBack_GPIO_Port, RightLimitBack_Pin);
	int RightLimitUp = HAL_GPIO_ReadPin(RightLimitUp_GPIO_Port, RightLimitUp_Pin);

	RightArm_State.ActualPosition = EncoderData->Encoder[4];

	if (RightArm_State.HomeState < Homed)
	{
		if (RightArm_State.HomeState == NotHomed)
		{
			RightArm_State.HomeCounter = 0;

			RGBLeds_SetAllColors(RightArm, Red, LED_On);
		}
		else if (RightArm_State.HomeState == Homing)
		{
			GenericArms_HAL_Brake(False, RightArm);
			RightArm_State.HomeCounter += 1;

			RightArm_State.Direction = Arm_Down;
			RightArm_State.PWM_Output = (100 - abs(20));

			if (RightLimitBack == 1)
			{
				// Set current encoder position to zero
				EncoderData->Encoder[4] = 0;

				RightArm_State.PWM_Output = (100 - abs(0));

				Generic_Arm_PositionSetpoint(RightArm, 1, 00);
				RightArm_State.HomeState = Homed;

				RGBLeds_SetAllColors(RightArm, Green, LED_On);
			}
		}
	}
	else if (RightArm_State.MotionState == Motion_Moving)
	{
		RightArm_State.ErrorPosition = RightArm_State.TargetPosition - RightArm_State.ActualPosition;

		RightSpeed = abs(RightArm_State.ErrorPosition/4);

		if (RightSpeed < 15) {RightSpeed = 15;}
		if (RightSpeed > 50) {RightSpeed = 50;}

		if (RightArm_State.ErrorPosition > 15)
		{
			RightArm_State.Direction = Arm_Up;
			RightArm_State.PWM_Output = (100 - abs(35));
			GenericArms_HAL_Brake(False, RightArm);
		}
		else if (RightArm_State.ErrorPosition < -15)
		{
			RightArm_State.Direction = Arm_Down;
			RightArm_State.PWM_Output = (100 - abs(35));
			GenericArms_HAL_Brake(False, RightArm);
		}
		else
		{
			RightArm_State.PWM_Output = (100 - abs(0));
			RightArm_State.MotionState = Motion_Breaking;
			RightArm_State.BrakeTimer = 0;
		}
	}
	else if (RightArm_State.MotionState == Motion_Breaking)
	{
		RightArm_State.PWM_Output = (100 - abs(0));
		GenericArms_HAL_Brake(True, RightArm);

		RightArm_State.BrakeTimer += 1;

		if (RightArm_State.BrakeTimer >= 1 * UPDATE_20HZ)
		{
			RightArm_State.MotionState = Motion_Idle;
		}
	}
	else if (RightArm_State.MotionState == Motion_Idle)
	{
		RightArm_State.PWM_Output = (100 - abs(0));
		GenericArms_HAL_Brake(False, RightArm);
	}


	HAL_TIM_Base_Start(LeftArm_State.TIM );
	HAL_TIM_PWM_Start(LeftArm_State.TIM , TIM_CHANNEL_1);

	HAL_TIM_Base_Start(RightArm_State.TIM );
	HAL_TIM_PWM_Start(RightArm_State.TIM , TIM_CHANNEL_2);

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
	LeftArm_State.MotionState = Motion_Idle;
	LeftArm_State.HomeState = NotHomed;

	LeftArm_State.TIM = htim;
	LeftArm_State.TIM_CHANNEL = TIM_CHANNEL_1;

	LeftArm_State.TargetPosition = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void LeftArm_Home()
{
	// Move forward a litte
	LeftArm_State.HomeState = Homing;
	LeftArm_State.HomeCounter = 0;
	RGBLeds_SetAllColors(LeftArm, Red, LED_On);
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
	RightArm_State.MotionState = Motion_Idle;
	RightArm_State.HomeState = NotHomed;

	RightArm_State.TIM = htim;
	RightArm_State.TIM_CHANNEL = TIM_CHANNEL_2;

	RightArm_State.TargetPosition = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void RightArm_Home()
{
	// Move forward a litte
	RightArm_State.HomeState = Homing;
	RightArm_State.HomeCounter = 0;
	RGBLeds_SetAllColors(RightArm, Red, LED_On);
}

void RightArm_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}
