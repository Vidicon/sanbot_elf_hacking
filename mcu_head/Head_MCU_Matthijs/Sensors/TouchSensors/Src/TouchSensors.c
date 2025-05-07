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

	TouchSensorData.Sensor[0] = !HAL_GPIO_ReadPin(Touch0_GPIO_Port, Touch0_Pin);
	TouchSensorData.Sensor[1] = !HAL_GPIO_ReadPin(Touch1_GPIO_Port, Touch1_Pin);
	TouchSensorData.Sensor[2] = !HAL_GPIO_ReadPin(Touch2_GPIO_Port, Touch2_Pin);
	TouchSensorData.Sensor[3] = !HAL_GPIO_ReadPin(Touch3_GPIO_Port, Touch3_Pin);
	TouchSensorData.Sensor[4] = !HAL_GPIO_ReadPin(Touch4_GPIO_Port, Touch4_Pin);
	TouchSensorData.Sensor[5] = !HAL_GPIO_ReadPin(Touch5_GPIO_Port, Touch5_Pin);
	TouchSensorData.Sensor[6] = !HAL_GPIO_ReadPin(Touch6_GPIO_Port, Touch6_Pin);
	TouchSensorData.Sensor[7] = !HAL_GPIO_ReadPin(Touch7_GPIO_Port, Touch7_Pin);
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


