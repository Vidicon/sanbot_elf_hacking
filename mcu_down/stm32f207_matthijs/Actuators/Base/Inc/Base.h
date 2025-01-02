#ifndef BASE_INC_BASE_H_
#define BASE_INC_BASE_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"
#include "Encoders.h"
#include "Compass.h"

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
	Base_Motion_Positive,
	Base_Motion_Negative
};

struct Base_State_Type {
	enum ENUM_BaseDirection Direction;
	TIM_HandleTypeDef *TIM;
	int PrevError;
	int AmplifierSetpoint;

	int TargetPosition;
	int SetpointDirection;
	int BrakeWindow;

	int Integral;
	int Differential;
//	int MainState;

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

	int PWM_Output;

	uint32_t TIM_CHANNEL;
	struct Encoders_Data_Type *EncoderPtr;
	};

void Base_Init(TIM_HandleTypeDef *htim9, TIM_HandleTypeDef *htim11, TIM_HandleTypeDef *htim12);

void Base_VelocitySetpoint(int Vx, int Vy, int PhiDot);

void Base_Update20Hz(struct Encoders_Data_Type *EncoderData);

void GenericBase_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void GenericBase_HAL_Direction(enum ENUM_BaseMotionState Direction, enum ENUM_BodyParts BodyPart);

void GenericBase_HAL_PWM(int PWM, enum ENUM_BodyParts BodyPart);

void Base_MotionControl(struct Compass_Sensor_Type *CompassData);

void Base_NewCompassRotation(char HighByte, char LowByte);

void Base_Brake(int inApplyBrake);

#endif


