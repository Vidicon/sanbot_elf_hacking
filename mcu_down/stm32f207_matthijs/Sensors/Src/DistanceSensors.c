/*
 * Distance_Sensors.c
 *
 *  Created on: Nov 1, 2024
 *      Author: matthijs
 */

#include "DistanceSensors.h"
#include <main.h>
#include <Right_Soft_I2C.h>
#include "protocol_0x55.h"

struct Distance_Sensor_Type DistanceData;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct Distance_Sensor_Type *DistanceSensors_GetPointer()
{
	return &DistanceData;
}


//------------------------------------------------------------------------------
// Functions to call from outside this file
//------------------------------------------------------------------------------
void DistanceSensors_Init()
{
	DistanceData.SelectedSensor = 0;
	DistanceSensors_Select(DistanceData.SelectedSensor);
}

void DistanceSensors_Update10Hz()
{
	Right_Soft_I2C_Write(0x40, 0x5E);
	DistanceData.Distance[DistanceData.SelectedSensor] = Right_Soft_I2C_Read(0x40);


	// Last step is to select the next sensor
	DistanceData.SelectedSensor += 1;
	if (DistanceData.SelectedSensor >= 4)
	{
		DistanceData.SelectedSensor = 0;

		// Send data to host
		SendDistanceSensors(DistanceSensors_GetPointer());
	}

	DistanceSensors_Select(DistanceData.SelectedSensor);
}


void DistanceSensors_Select(int SensorID)
{
	if (SensorID == 0) { HAL_GPIO_WritePin(EN1_Distance_J26_GPIO_Port, EN1_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J26_GPIO_Port, EN1_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 1) { HAL_GPIO_WritePin(EN2_Distance_J26_GPIO_Port, EN2_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN2_Distance_J26_GPIO_Port, EN2_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 2) { HAL_GPIO_WritePin(EN3_Distance_J26_GPIO_Port, EN3_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN3_Distance_J26_GPIO_Port, EN3_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 3) { HAL_GPIO_WritePin(EN4_Distance_J26_GPIO_Port, EN4_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN4_Distance_J26_GPIO_Port, EN4_Distance_J26_Pin, GPIO_PIN_RESET); }
}

