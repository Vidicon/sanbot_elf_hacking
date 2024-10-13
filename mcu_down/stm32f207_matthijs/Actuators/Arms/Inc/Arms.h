#ifndef ARMS_INC_ARMS_H_
#define ARMS_INC_ARMS_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"
#include "Encoders.h"

enum ENUM_ArmHomeState {
	Arm_NotHomed,
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
	Arm_Motion_MovingDown
};

struct Arm_State_Type {
	enum ENUM_ArmMotionState MotionState;
	enum ENUM_ArmDirection ArmDirection;
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

void GenericArm_HAL_Brake(enum ENUM_Booleans BrakeEnable, enum ENUM_BodyParts BodyPart);

void GenericArm_Update20Hz(struct Encoders_Data_Type EncoderData, struct Arm_State_Type *Arm_State, enum ENUM_BodyParts BodyPart);

void GenericArm_HAL_Direction(enum ENUM_Booleans Up, enum ENUM_BodyParts BodyPart);

void GenericArm_Init(struct Arm_State_Type LeftArm_State);

//------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim);

void LeftArm_SelfTest(enum ENUM_Booleans Enabled);

void LeftArm_Update20Hz(struct Encoders_Data_Type EncoderData);

void LeftArm_HAL_Brake(enum ENUM_Booleans BrakeEnable);

void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable);

void LeftArm_NewSetpoint(int NewSetpoint);

//------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim);

void RightArm_SelfTest(enum ENUM_Booleans Enabled);

void RightArm_Update20Hz(struct Encoders_Data_Type EncoderData);

void RightArm_HAL_Brake(enum ENUM_Booleans BrakeEnable);

void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable);

void RightArm_NewSetpoint(int NewSetpoint);

//------------------------------------------------
#endif /* ARMS_INC_ARMS_H_ */


