#include "HeadMotors.h"
#include "stm32f1xx.h"
#include <main.h>
#include "Encoders.h"
#include <stdlib.h>  // For abs()
#include "RGBLeds_Head.h"

struct HeadMotor_State_Type HeadPan_State;
struct HeadMotor_State_Type HeadTilt_State;

void Generic_Head_Position_Setpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte)
{
	short combined = ((unsigned char)HighByte << 8) | (unsigned char)LowByte;

	if (BodyPart == HeadPan)
	{
		HeadPan_State.TargetPosition = combined;
		Generic_Head_HAL_Brake(False, BodyPart);
		HeadPan_State.MotionState = Motion_Moving;
	}

	if (BodyPart == HeadTilt)
	{
		HeadTilt_State.TargetPosition = combined;
		Generic_Head_HAL_Brake(False, BodyPart);
		HeadTilt_State.MotionState = Motion_Moving;
	}
}

void Generic_Head_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart)
{
	//	 Works inverted. High to release brake.
	if (BodyPart == HeadPan)
	{
		HAL_GPIO_WritePin(PanEnable_GPIO_Port, PanEnable_Pin, !BrakeEnable);
	}

	if (BodyPart == HeadTilt)
	{
		HAL_GPIO_WritePin(TiltEnable_GPIO_Port, TiltEnable_Pin, !BrakeEnable);
	}
}

void GenericHead_HAL_Direction(enum ENUM_HeadMotorDirection Direction, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == HeadPan)
	{
		if (Direction == Move_Pos)
		{
			HAL_GPIO_WritePin(PanDirection_GPIO_Port, PanDirection_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(PanDirection_GPIO_Port, PanDirection_Pin, GPIO_PIN_SET);
		}
	}

	if (BodyPart == HeadTilt)
	{
		if (Direction == Move_Pos)
		{
			HAL_GPIO_WritePin(TiltDirection_GPIO_Port, TiltDirection_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(TiltDirection_GPIO_Port, TiltDirection_Pin, GPIO_PIN_RESET);
		}
	}
}

void Generic_Head_HAL_PWM(int PWM, enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == HeadPan)
	{
		__HAL_TIM_SET_COMPARE(HeadPan_State.TIM, HeadPan_State.TIM_CHANNEL, PWM);
	}

	if (BodyPart == HeadTilt)
	{
		__HAL_TIM_SET_COMPARE(HeadTilt_State.TIM, HeadTilt_State.TIM_CHANNEL, PWM);
	}
}

void Head_Update20Hz(struct Encoders_Data_Type *EncoderData)
{
	//--------------------------------------------------------------------------------
	// Head pan
	//--------------------------------------------------------------------------------
	int HeadPanLimitNeg = HAL_GPIO_ReadPin(PanNegSensor_GPIO_Port,PanNegSensor_Pin);
//	int HeadPanLimitPos = HAL_GPIO_ReadPin(PanPosSensor_GPIO_Port, PanPosSensor_Pin);

	int PanSpeed = 30;

	HeadPan_State.ActualPosition = EncoderData->Encoder[1];

	if (HeadPan_State.ActualPosition >= 32768)
	{
		HeadPan_State.ActualPosition = EncoderData->Encoder[1] - 65536;
	}

	if (HeadPan_State.HomeState < Homed)
	{
		if (HeadPan_State.HomeState == NotHomed)
		{
			HeadPan_State.HomeCounter = 0;
		}
		else if (HeadPan_State.HomeState == Homing)
		{
			Generic_Head_HAL_Brake(False, HeadPan);
			HeadPan_State.HomeCounter += 1;

			HeadPan_State.Direction = Move_Neg;
			HeadPan_State.PWM_Output = (100 - abs(30));

			if (HeadPanLimitNeg == 1)
			{
				// Set current encoder position to zero
				__HAL_TIM_SET_COUNTER(EncoderData->TIM[1], 0);

				Generic_Head_Position_Setpoint(HeadPan, 1, 170);
				HeadPan_State.HomeState = Homed;

				RGBLeds_SetAllColors(LeftHead, Green, LED_On);
			}
		}
	}
	else if (HeadPan_State.MotionState == Motion_Moving)
	{
		HeadPan_State.ErrorPosition = HeadPan_State.TargetPosition - HeadPan_State.ActualPosition;

		if (HeadPan_State.ErrorPosition > 10)
		{
			HeadPan_State.Direction = Move_Pos;
			HeadPan_State.PWM_Output = (100 - abs(PanSpeed));
			Generic_Head_HAL_Brake(False, HeadPan);
		}
		else if (HeadPan_State.ErrorPosition < -10)
		{
			HeadPan_State.Direction = Move_Neg;
			HeadPan_State.PWM_Output = (100 - abs(PanSpeed));
			Generic_Head_HAL_Brake(False, HeadPan);
		}
		else
		{
			HeadPan_State.PWM_Output = (100 - abs(0));
			HeadPan_State.MotionState = Motion_Breaking;
			HeadPan_State.BrakeTimer = 0;
		}
	}
	else if (HeadPan_State.MotionState == Motion_Breaking)
	{
		HeadPan_State.PWM_Output = (100 - abs(0));
		Generic_Head_HAL_Brake(True, HeadPan);

		HeadPan_State.BrakeTimer += 1;

		if (HeadPan_State.BrakeTimer >= 1 * UPDATE_20HZ)
		{
			HeadPan_State.MotionState = Motion_Idle;
		}
	}
	else if (HeadPan_State.MotionState == Motion_Idle)
	{
		HeadPan_State.PWM_Output = (100 - abs(0));
		Generic_Head_HAL_Brake(False, HeadPan);
	}

	//--------------------------------------------------------------------------------
	// Head Tilt
	//--------------------------------------------------------------------------------
	int HeadTiltLimitNeg = HAL_GPIO_ReadPin(TiltNegSensor_GPIO_Port, TiltNegSensor_Pin);
//	int HeadTiltLimitPos = HAL_GPIO_ReadPin(TiltPosSensor_GPIO_Port, TiltPosSensor_Pin);

	int TiltSpeed = 20;

	HeadTilt_State.ActualPosition = EncoderData->Encoder[0];

	if (HeadTilt_State.ActualPosition  < -32768)
	{
		HeadTilt_State.ActualPosition = EncoderData->Encoder[0] + 65536;
	}

	if (HeadTilt_State.HomeState < Homed)
	{
		if (HeadTilt_State.HomeState == NotHomed)
		{
			HeadTilt_State.HomeCounter = 0;
		}
		else if (HeadTilt_State.HomeState == Homing)
		{
			Generic_Head_HAL_Brake(False, HeadTilt);
			HeadTilt_State.HomeCounter += 1;

			HeadTilt_State.Direction = Move_Neg;
			HeadTilt_State.PWM_Output = (100 - abs(30));

			if (HeadTiltLimitNeg == 1)
			{
				// Set current encoder position to zero
				__HAL_TIM_SET_COUNTER(EncoderData->TIM[0], 0);

				Generic_Head_Position_Setpoint(HeadTilt, 0, 200);
				HeadTilt_State.HomeState = Homed;

				RGBLeds_SetAllColors(RightHead, Green, LED_On);
			}
		}
	}
	else if (HeadTilt_State.MotionState == Motion_Moving)
	{
		HeadTilt_State.ErrorPosition = HeadTilt_State.TargetPosition - HeadTilt_State.ActualPosition;

		if (HeadTilt_State.ErrorPosition > 10)
		{
			HeadTilt_State.Direction = Move_Pos;
			HeadTilt_State.PWM_Output = (100 - abs(TiltSpeed));
			Generic_Head_HAL_Brake(False, HeadTilt);
		}
		else if (HeadTilt_State.ErrorPosition < -10)
		{
			HeadTilt_State.Direction = Move_Neg;
			HeadTilt_State.PWM_Output = (100 - abs(TiltSpeed));
			Generic_Head_HAL_Brake(False, HeadTilt);
		}
		else
		{
			HeadTilt_State.PWM_Output = (100 - abs(0));
			HeadTilt_State.MotionState = Motion_Breaking;
			HeadTilt_State.BrakeTimer = 0;
		}
	}
	else if (HeadTilt_State.MotionState == Motion_Breaking)
	{
		HeadTilt_State.PWM_Output = (100 - abs(0));
		Generic_Head_HAL_Brake(True, HeadTilt);

		HeadTilt_State.BrakeTimer += 1;

		if (HeadTilt_State.BrakeTimer >= 1 * UPDATE_20HZ)
		{
			HeadTilt_State.MotionState = Motion_Idle;
		}
	}
	else if (HeadTilt_State.MotionState == Motion_Idle)
	{
		HeadTilt_State.PWM_Output = (100 - abs(0));
		Generic_Head_HAL_Brake(False, HeadTilt);
	}

	GenericHead_HAL_Direction(HeadPan_State.Direction, HeadPan);
	GenericHead_HAL_Direction(HeadTilt_State.Direction, HeadTilt);

	Generic_Head_HAL_PWM(HeadPan_State.PWM_Output, HeadPan);
	Generic_Head_HAL_PWM(HeadTilt_State.PWM_Output, HeadTilt);
}

//----------------------------------------------------------------
// Head Pan Init
//----------------------------------------------------------------
void Head_Pan_Init(TIM_HandleTypeDef *htim)
{
	HeadPan_State.MotionDirection = Move_Pos;
	HeadPan_State.MotionState = Motion_Idle;
	HeadPan_State.HomeState = NotHomed;

	// Encoder counter timer
	HeadPan_State.TIM = htim;
	HeadPan_State.TIM_CHANNEL = TIM_CHANNEL_4;

	HeadPan_State.TargetPosition = 0;

	// PWM generation timer
	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

	// 100 = not moving
	__HAL_TIM_SET_COMPARE(HeadPan_State.TIM, HeadPan_State.TIM_CHANNEL, 100);
}

void Head_Pan_Home()
{
	// Move forward a litte
	HeadPan_State.HomeState = Homing;
	HeadPan_State.HomeCounter = 0;

	RGBLeds_SetAllColors(LeftHead, Red, LED_On);
}

void Head_Pan_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}

//----------------------------------------------------------------
// Head Tilt Init
//----------------------------------------------------------------
void Head_Tilt_Init(TIM_HandleTypeDef *htim)
{
	HeadTilt_State.MotionDirection = Move_Pos;
	HeadTilt_State.MotionState = Motion_Idle;
	HeadTilt_State.HomeState = NotHomed;

	// Encoder counter timer
	HeadTilt_State.TIM = htim;
	HeadTilt_State.TIM_CHANNEL = TIM_CHANNEL_3;

	HeadTilt_State.TargetPosition = 0;

	// PWM generation timer
	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

	// 100 = not moving
	__HAL_TIM_SET_COMPARE(HeadTilt_State.TIM, HeadTilt_State.TIM_CHANNEL, 100);
}

void Head_Tilt_Home()
{
	// Move forward a litte
	HeadTilt_State.HomeState = Homing;
	HeadTilt_State.HomeCounter = 0;

	RGBLeds_SetAllColors(RightHead, Red, LED_On);
}

void Head_Tilt_SelfTest(enum ENUM_Booleans Enabled)
{
	// To do
}
