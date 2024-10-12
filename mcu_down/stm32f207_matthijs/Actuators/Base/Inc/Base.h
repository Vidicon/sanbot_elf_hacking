/*
 * Arms.h
 *
 *  Created on: May 3, 2024
 *      Author: matthijs
 */

#ifndef BASE_INC_H_
#define BASE_INC_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"

enum ENUM_Base_HomeState {
	BaseNotHomed,
	BaseHomed
};

enum ENUM_Base_MotionState {
	BaseMotion_Disabled,
	BaseMotion_AtTarget,
	BaseMotion_Error,
	BaseMotion_MovingUp,
	BaseMotion_MovingDown
};

struct Base_State_Type {
	enum ENUM_Base_HomeState Homed;
	enum ENUM_Base_MotionState MotionState;
	int Angle;
	int Timer;
	int SelTestRunning;
	int Speed;
	TIM_HandleTypeDef *TIM;
	};

//------------------------------------------------
void Base_Init(TIM_HandleTypeDef *htim);

void Base_SelfTest();

#endif
