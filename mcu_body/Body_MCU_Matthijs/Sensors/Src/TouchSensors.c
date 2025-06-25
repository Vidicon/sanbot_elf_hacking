/*
 * RGBLeds.c
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */
#include "TouchSensors.h"
#include "RobotGlobals.h"
#include "main.h"

struct TouchSensors_Data_Type TouchSensorData;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct TouchSensors_Data_Type *TouchSensors_GetPointer()
{
	return &TouchSensorData;
}

void TouchSensors_Init()
{
	for (int i = 0; i < NO_TOUCH_SENSORS; i++)
	{
		TouchSensorData.Sensor[i] = False;
		TouchSensorData.OldSensor[i] = False;
	}
}

void TouchSensors_Update()
{
	for (int i = 0; i < NO_TOUCH_SENSORS; i++)
	{
		TouchSensorData.OldSensor[i] = TouchSensorData.Sensor[i];
	}

	TouchSensorData.Sensor[0] = !HAL_GPIO_ReadPin(TouchL_GPIO_Port, TouchL_Pin);
	TouchSensorData.Sensor[1] = !HAL_GPIO_ReadPin(TouchR_GPIO_Port, TouchR_Pin);
}

int TouchSensor_AnyPressed()
{
	int result = 0;

	for (int i = 0; i < NO_TOUCH_SENSORS; i++)
	{
		if ((TouchSensorData.OldSensor[i] == 0) && (TouchSensorData.Sensor[i] == 1))
		{
			result = 1;
		}
	}

	return result;
}

int TouchSensor_AnyChanged()
{
	int result = 0;

	for (int i = 0; i < NO_TOUCH_SENSORS; i++)
	{
		if (TouchSensorData.OldSensor[i] != TouchSensorData.Sensor[i])
		{
			result = 1;
		}
	}

	return result;
}


