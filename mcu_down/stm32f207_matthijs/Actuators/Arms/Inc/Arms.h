/*
 * Arms.h
 *
 *  Created on: May 3, 2024
 *      Author: matthijs
 */

#ifndef ARMS_INC_ARMS_H_
#define ARMS_INC_ARMS_H_

#include "RobotGlobals.h"


enum ENUM_HomeState {
	NotHomed,
	Homed
};

enum ENUM_ArmDirection {
	Arm_Up,
	Arm_Down
};

enum ENUM_MotionState {
	Motion_Disabled,
	Motion_Moving,
	Motion_AtTarget,
	Motion_Error
};

struct Arm_State_Type {
	enum ENUM_HomeState Homed;
	enum ENUM_MotionState MotionState;
	int Angle;
	enum ENUM_ArmDirection Direction;
	};

//------------------------------------------------
void LeftArm_Init();

void LeftArm_SelfTest();

void LeftArm_MoveToAngle(int TargetAngle);

void LeftArm_Update10Hz();

//------------------------------------------------
void RightArm_Init();

void RightArm_SelfTest();

void RightArm_MoveToAngle(int TargetAngle);

void RightArm_Update10Hz();

//------------------------------------------------
void GenericArm_Init(struct Arm_State_Type LeftArm_State);

#endif /* ARMS_INC_ARMS_H_ */


