#ifndef BASE_INC_BASE_H_
#define BASE_INC_BASE_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"
#include "Encoders.h"

enum ENUM_BaseHomeState {
	Base_NotHomed,
	Base_Homed
};

enum ENUM_BaseDirection {
	Base_Fwd,
	Base_Rev
};

enum ENUM_BaseMotionState {
	Base_Motion_Disabled,
	Base_Motion_AtTarget,
	Base_Motion_Error,
	Base_Motion_MovingUp,
	Base_Motion_MovingDown
};

struct Base_State_Type {
	enum ENUM_BaseMotionState MotionState;
	enum ENUM_BaseDirection Direction;
	TIM_HandleTypeDef *TIM;
	int PrevError;
	int AmplifierSetpoint;

	int ActualPosition;
	int TargetPosition;
	int SetpointPosition;
	int SetpointDirection;
	int BrakeWindow;

	int ErrorPrev;
	int Integral;
	int Differential;

	int Error;
	int Output;

	int MainState;

	uint32_t TIM_CHANNEL;

	struct Encoders_Data_Type *EncoderPtr;
	};

void Base_Init(TIM_HandleTypeDef *htim9, TIM_HandleTypeDef *htim11);

void Base_Update20Hz(struct Encoders_Data_Type EncoderData, struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart);

void Base_VelocitySetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte);


void CenterBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData);

void LeftBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData);

void RightBaseMotor_Update20Hz(struct Encoders_Data_Type EncoderData);

void GenericBase_Update20Hz(struct Encoders_Data_Type EncoderData, struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart);

void GenericBase_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void Temp(int speed);

#endif


