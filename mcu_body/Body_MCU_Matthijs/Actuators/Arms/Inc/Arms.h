#ifndef ARMS_INC_ARMS_H_
#define ARMS_INC_ARMS_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"
#include "Encoders.h"

extern struct Arm_State_Type LeftArm_State;
extern struct Arm_State_Type RightArm_State;

enum ENUM_ArmHomeState {
	Arm_NotHomed,
	Arm_Homing,
	Arm_Homed
};

enum ENUM_ArmDirection {
	Arm_Up,
	Arm_Down
};

enum ENUM_ArmMotionState {
	Arm_Motion_Disabled,
	Arm_Motion_AtTarget,
	Arm_Motion_Error,
	Arm_Motion_MovingUp,
	Arm_Motion_MovingDown,
};

struct Arm_State_Type {
	enum ENUM_ArmMotionState MotionState;
	enum ENUM_ArmDirection ArmDirection;
	enum ENUM_ArmHomeState HomeState;

	TIM_HandleTypeDef *TIM;

//	int PrevError;
//	int AmplifierSetpoint;
//
//	int SetpointDirection;
//	int BrakeWindow;
//
//	int ErrorPositionPrev;
//	int Integral;
//	int Differential;
//
//	int Output;
//
//	int MainState;

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

void Arm_PositionSetpoint(enum ENUM_BodyParts BodyPart, char HighByte, char LowByte);

void GenericArms_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void Arms_Update20Hz(struct Encoders_Data_Type *EncoderData);

void GenericArms_HAL_Direction(enum ENUM_ArmMotionState Direction, enum ENUM_BodyParts BodyPart);

void GenericArms_Init(struct Arm_State_Type LeftArm_State);



//------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim);

void LeftArm_SelfTest(enum ENUM_Booleans Enabled);

void LeftArm_Home();

//void LeftArm_Update20Hz(struct Encoders_Data_Type *EncoderData);
//
//void LeftArm_HAL_Brake(enum ENUM_Booleans BrakeEnable);
//
//void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable);
//
//void LeftArm_NewSetpoint(int NewSetpoint);

//------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim);

void RightArm_SelfTest(enum ENUM_Booleans Enabled);

void RightArm_Home();

//void RightArm_Update20Hz(struct Encoders_Data_Type *EncoderData);
//
//void RightArm_HAL_Brake(enum ENUM_Booleans BrakeEnable);
//
//void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable);
//
//void RightArm_NewSetpoint(int NewSetpoint);

//------------------------------------------------
#endif /* ARMS_INC_ARMS_H_ */


