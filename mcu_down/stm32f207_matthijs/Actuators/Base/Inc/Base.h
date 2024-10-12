/*
 * Arms.h
 *
 *  Created on: May 3, 2024
 *      Author: matthijs
 */

#ifndef BASE_INC_H__
#define BASE_INC_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"



struct Base_State_Type {
	enum ENUM_HomeState Homed;
	enum ENUM_MotionState MotionState;
	enum ENUM_ArmDirection Direction;
	int Angle;
	int Timer;
	int SelTestRunning;
	int Speed;
	TIM_HandleTypeDef *TIM;
	};

//------------------------------------------------
void Base_Init(TIM_HandleTypeDef *htim);

void Base_SelfTest();
