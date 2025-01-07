#ifndef HEADMOTOR_INC_H_
#define HEADMOTOR_INC_H_

#include "RobotGlobals.h"
#include "stm32f1xx_hal.h"
#include "Encoders.h"

// Allocate the memory in the c-file
// Make available for other c-files to use, in the h-file
extern struct HeadMotor_State_Type HeadPan_State;
extern struct HeadMotor_State_Type HeadTilt_State;

enum ENUM_HeadHomeMotorState {
	NotHomed,
	Homing,
	Homed
};

enum ENUM_HeadMotorDirection {
	Move_Pos,
	Move_Neg
};

enum ENUM_HeadMotorMotionState {
	Motion_Idle,
	Motion_Moving
};

struct HeadMotor_State_Type {
	enum ENUM_HeadHomeMotorState HomeState ;
	enum ENUM_HeadMotorDirection MotionDirection;
	enum ENUM_HeadMotorMotionState MotionState;

	TIM_HandleTypeDef *TIM;

	int VelocitySetpoint;
	int Direction;
	int PWM_Output;

	int ActualPosition;
	int TargetPosition;
	int SetpointPosition;
	int ErrorPosition;

	int HomeCounter;

	uint32_t TIM_CHANNEL;
	struct Encoders_Data_Type *EncoderPtr;
	};

void Generic_Head_Position_Setpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte);

void Generic_Head_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void GenericHead_HAL_Direction(enum ENUM_HeadMotorDirection Direction, enum ENUM_BodyParts BodyPart);

void Generic_Head_HAL_PWM(int PWM, enum ENUM_BodyParts BodyPart);

void Head_Update20Hz(struct Encoders_Data_Type *EncoderData);

//------------------------------------------------
// PAN
//------------------------------------------------
void Head_Pan_Init(TIM_HandleTypeDef *htim);

void Head_Pan_Home();

void Head_Pan_SelfTest(enum ENUM_Booleans Enabled);

//------------------------------------------------
// TILT
//------------------------------------------------

//------------------------------------------------
#endif /* HEADMOTOR_INC_H_ */


