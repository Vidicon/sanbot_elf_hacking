/*
 * Arms.h
 *
 *  Created on: May 3, 2024
 *      Author: matthijs
 */

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
	enum ENUM_ArmHomeState Homed;
	enum ENUM_ArmMotionState MotionState;
	enum ENUM_ArmDirection Direction;
	int Angle;
	int Timer;
	int SelTestRunning;
	int Speed;
	TIM_HandleTypeDef *TIM;
	int PrevError;
	int AmplifierSetpoint;

	int SetpointState;
	int ActualPosition;
	int ActualPositionPrev;
	int TargetPosition;
	int SetpointPosition;
	int SetpointDirection;

	int ErrorPrev;
	int Integral;
	int Differential;

	struct Encoders_Data_Type *EncoderPtr;
	};

//------------------------------------------------
void LeftArm_Init(TIM_HandleTypeDef *htim);

void LeftArm_SelfTest(enum ENUM_Booleans Enabled);

void LeftArm_MoveToAngle(int TargetAngle);

void LeftArm_Update10Hz(struct Encoders_Data_Type EncoderData);

void LeftArm_EnableBrake(enum ENUM_Booleans BrakeEnable);

void LeftArm_NewSetpoint(int NewSetpoint);

//------------------------------------------------
void RightArm_Init(TIM_HandleTypeDef *htim);

void RightArm_SelfTest(enum ENUM_Booleans Enabled);

void RightArm_MoveToAngle(int TargetAngle);

void RightArm_Update10Hz();

void RightArm_EnableBrake(enum ENUM_Booleans BrakeEnable);


//------------------------------------------------
void GenericArm_Init(struct Arm_State_Type LeftArm_State);

#endif /* ARMS_INC_ARMS_H_ */


