#ifndef MOTION_SENSORS_H_
#define MOTION_SENSORS_H_

#include "RobotGlobals.h"
#include "stm32f2xx_hal.h"

struct MotionSensors_Data_Type {
	int PreviousValue[2];
	int CurrentValue[2];
	};

struct MotionSensors_Data_Type *MotionSensors_GetPointer();

void MotionSensors_Init();

void MotionSensors_Update10Hz();


//------------------------------------------------
#endif /* MOTION_SENSORS_H_ */


