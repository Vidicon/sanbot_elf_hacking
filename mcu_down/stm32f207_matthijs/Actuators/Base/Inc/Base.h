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

	int TargetPosition;
	int SetpointDirection;
	int BrakeWindow;

	int Integral;
	int Differential;


	int MainState;

	// Velocity controller
	int TargetVelocity;
	int TargetAcceleration;


	int SetpointVelocity;
	int ActualVelocity;
	int ActualVelocity_Prev;
	int ActualPosition_Prev;

	int ErrorPosition;
	int ErrorPositionPrev;
	int ErrorVelocity;
	int ErrorVelocityInt;

	int ActualPosition;
	int SetpointPosition;

	int NewData;

	int PID_Output;

	int Logging1;
	int Logging2;

	uint32_t TIM_CHANNEL;
	struct Encoders_Data_Type *EncoderPtr;
	};

void Base_Init(TIM_HandleTypeDef *htim9, TIM_HandleTypeDef *htim11);

void Base_Update20Hz(struct Encoders_Data_Type EncoderData, struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart);

//void Base_VelocitySetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte);

void Base_VelocitySetpoint(int Vx, int Vy, int PhiDot, int Acceleration);

void CenterBaseMotor_Update20Hz(struct Encoders_Data_Type *EncoderData);

//void LeftBaseMotor_Update20Hz(struct Encoders_Data_Type *EncoderData);
//
//void RightBaseMotor_Update20Hz(struct Encoders_Data_Type *EncoderData);

void GenericBase_Update20Hz(struct Base_State_Type *Base_State, enum ENUM_BodyParts BodyPart);

void GenericBase_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void GenericBase_HAL_Direction(enum ENUM_Booleans Left, enum ENUM_BodyParts BodyPart);

void Temp(int speed);

void TracingUpdate();

#endif


