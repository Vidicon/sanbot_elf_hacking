#include "Arms.h"
#include "stm32f2xx.h"
#include <main.h>

struct Arm_State_Type LeftArm_State;
struct Arm_State_Type RightArm_State;

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim)
{
	LeftArm_State.Angle = 0;
	LeftArm_State.Direction = Arm_Up;
	LeftArm_State.Homed = Arm_NotHomed;
	LeftArm_State.MotionState = Arm_Motion_Disabled;
	LeftArm_State.Timer = 0;
	LeftArm_State.SelTestRunning = 0;
	LeftArm_State.TIM = htim;
	LeftArm_State.Speed = 0;


	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void LeftArm_SelfTest(enum ENUM_Booleans Enabled)
{
	if (Enabled)
	{
		LeftArm_EnableBrake(False);
		LeftArm_State.SelTestRunning = 1;
		LeftArm_State.MotionState = Arm_Motion_MovingUp;
		LeftArm_State.Speed = 10;
	}
	else
	{
		LeftArm_EnableBrake(True);
		LeftArm_State.SelTestRunning = 0;
		LeftArm_State.MotionState = Arm_Motion_Disabled;
		LeftArm_State.Speed = 0;
	}
}

void LeftArm_MoveToAngle(int TargetAngle)
{
	// Move if correctly homed
}

void LeftArm_Update10Hz()
{
	//  Check if selftest is running
	if (LeftArm_State.SelTestRunning == 1)
	{
		LeftArm_State.Timer += 1;

		if (LeftArm_State.Timer <= 1 * UPDATE_10HZ)
		{
			LeftArm_State.MotionState = Arm_Motion_MovingUp;
		}
		else if (LeftArm_State.Timer <= 2 * UPDATE_10HZ)
		{
			LeftArm_State.MotionState = Arm_Motion_MovingDown;
		}
		else
		{
			LeftArm_State.MotionState = Arm_Motion_AtTarget;
		}
	}
	else
	{

	}

	// Write correct outputs
	if (LeftArm_State.MotionState == Arm_Motion_MovingUp)
	{
		__HAL_TIM_SET_COMPARE(LeftArm_State.TIM, TIM_CHANNEL_1, LeftArm_State.Speed);

		HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_SET);

		LeftArm_EnableBrake(False);
	}
	else if (LeftArm_State.MotionState == Arm_Motion_MovingDown)
	{
		__HAL_TIM_SET_COMPARE(LeftArm_State.TIM, TIM_CHANNEL_1, LeftArm_State.Speed);

		HAL_GPIO_WritePin(LeftArmUp_GPIO_Port, LeftArmUp_Pin, GPIO_PIN_RESET);

		LeftArm_EnableBrake(False);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(LeftArm_State.TIM, TIM_CHANNEL_1, 0);
	}
}

void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
{
	// Works inverted. High to release brake.
	HAL_GPIO_WritePin(LeftArmBrake_GPIO_Port, LeftArmBrake_Pin, !BrakeEnable);
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim)
{
	RightArm_State.Angle = 0;
	RightArm_State.Direction = Arm_Up;
	RightArm_State.Homed = Arm_NotHomed;
	RightArm_State.MotionState = Arm_Motion_Disabled;
	RightArm_State.Timer = 0;
	RightArm_State.SelTestRunning = 0;
	RightArm_State.TIM = htim;
	RightArm_State.Speed = 0;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void RightArm_SelfTest(enum ENUM_Booleans Enabled)
{
	if (Enabled)
	{
		RightArm_EnableBrake(False);
		RightArm_State.SelTestRunning = 1;
		RightArm_State.MotionState = Arm_Motion_MovingUp;
		RightArm_State.Speed = 10;
	}
	else
	{
		RightArm_EnableBrake(True);
		RightArm_State.SelTestRunning = 0;
		RightArm_State.MotionState = Arm_Motion_Disabled;
		RightArm_State.Speed = 0;
	}
}

void RightArm_MoveToAngle(int TargetAngle)
{

}

void RightArm_Update10Hz()
{
	//  Check if selftest is running
	if (RightArm_State.SelTestRunning == 1)
	{
		RightArm_State.Timer += 1;

		if (RightArm_State.Timer <= 1 * UPDATE_10HZ)
		{
			RightArm_State.MotionState = Arm_Motion_MovingDown;
		}
		else if (RightArm_State.Timer <= 2 * UPDATE_10HZ)
		{
			RightArm_State.MotionState = Arm_Motion_MovingUp;
		}
		else
		{
			RightArm_State.MotionState = Arm_Motion_AtTarget;
		}
	}
	else
	{

	}

	// Write correct outputs
	if (RightArm_State.MotionState == Arm_Motion_MovingUp)
	{
		__HAL_TIM_SET_COMPARE(RightArm_State.TIM, TIM_CHANNEL_2, RightArm_State.Speed);

		HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_SET);

		RightArm_EnableBrake(False);
	}
	else if (RightArm_State.MotionState == Arm_Motion_MovingDown)
	{
		__HAL_TIM_SET_COMPARE(RightArm_State.TIM, TIM_CHANNEL_2, RightArm_State.Speed);

		HAL_GPIO_WritePin(RightArmUp_GPIO_Port, RightArmUp_Pin, GPIO_PIN_RESET);

		RightArm_EnableBrake(False);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(RightArm_State.TIM, TIM_CHANNEL_2, 0);
	}
}

void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable)
{
	// Works inverted. High to release brake.
	HAL_GPIO_WritePin(RightArmBrake_GPIO_Port, RightArmBrake_Pin, !BrakeEnable);
}

