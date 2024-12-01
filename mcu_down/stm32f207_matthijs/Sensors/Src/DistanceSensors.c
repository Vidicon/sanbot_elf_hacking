/*
 * Distance_Sensors.c
 *
 *  Created on: Nov 1, 2024
 *      Author: matthijs
 */

#include "DistanceSensors.h"
#include <main.h>
#include <Left_Soft_I2C.h>
#include <Right_Soft_I2C.h>
#include <Mid_Soft_I2C.h>
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

void DistanceSensors_Update20Hz()
{
	// Left
	if (DistanceData.SelectedSensor <= 3)
	{

		Left_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Left_Soft_I2C_Read(0x40);
	}
	// Right
	else if (DistanceData.SelectedSensor <= 3+4)
	{
		Right_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Right_Soft_I2C_Read(0x40);
	}
	// Center
	else
	{
		Mid_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Mid_Soft_I2C_Read(0x40);
	}

	// Last step is to select the next sensor
	DistanceData.SelectedSensor += 1;
	if (DistanceData.SelectedSensor > 10)
	{
		DistanceData.SelectedSensor = 0;

		// Send data to host
		SendDistanceSensors(DistanceSensors_GetPointer());
	}

	DistanceSensors_Select(DistanceData.SelectedSensor);
}


void DistanceSensors_Select(int SensorID)
{
	//-----------------------------------------------------------------------------------------------
	// 4 Left side sensors
	//-----------------------------------------------------------------------------------------------
	if (SensorID == 0) { HAL_GPIO_WritePin(EN1_Distance_J18_GPIO_Port, EN1_Distance_J18_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J18_GPIO_Port, EN1_Distance_J18_Pin, GPIO_PIN_RESET); }

	if (SensorID == 1) { HAL_GPIO_WritePin(EN2_Distance_J18_GPIO_Port, EN2_Distance_J18_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN2_Distance_J18_GPIO_Port, EN2_Distance_J18_Pin, GPIO_PIN_RESET); }

	if (SensorID == 2) { HAL_GPIO_WritePin(EN3_Distance_J18_GPIO_Port, EN3_Distance_J18_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN3_Distance_J18_GPIO_Port, EN3_Distance_J18_Pin, GPIO_PIN_RESET); }

	if (SensorID == 3) { HAL_GPIO_WritePin(EN4_Distance_J18_GPIO_Port, EN4_Distance_J18_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN4_Distance_J18_GPIO_Port, EN4_Distance_J18_Pin, GPIO_PIN_RESET); }

	//-----------------------------------------------------------------------------------------------
	// 4 Right side sensors
	//-----------------------------------------------------------------------------------------------
	if (SensorID == 4) { HAL_GPIO_WritePin(EN1_Distance_J26_GPIO_Port, EN1_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J26_GPIO_Port, EN1_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 5) { HAL_GPIO_WritePin(EN2_Distance_J26_GPIO_Port, EN2_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN2_Distance_J26_GPIO_Port, EN2_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 6) { HAL_GPIO_WritePin(EN3_Distance_J26_GPIO_Port, EN3_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN3_Distance_J26_GPIO_Port, EN3_Distance_J26_Pin, GPIO_PIN_RESET); }

	if (SensorID == 7) { HAL_GPIO_WritePin(EN4_Distance_J26_GPIO_Port, EN4_Distance_J26_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN4_Distance_J26_GPIO_Port, EN4_Distance_J26_Pin, GPIO_PIN_RESET); }

	//-----------------------------------------------------------------------------------------------
	// 3 Center sensors
	//-----------------------------------------------------------------------------------------------
	if (SensorID == 8) { HAL_GPIO_WritePin(EN1_Distance_J21_GPIO_Port, EN1_Distance_J21_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J21_GPIO_Port, EN1_Distance_J21_Pin, GPIO_PIN_RESET); }

	if (SensorID == 9) { HAL_GPIO_WritePin(EN1_Distance_J24_GPIO_Port, EN1_Distance_J24_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J24_GPIO_Port, EN1_Distance_J24_Pin, GPIO_PIN_RESET); }

	if (SensorID == 10) { HAL_GPIO_WritePin(EN1_Distance_J28_GPIO_Port, EN1_Distance_J28_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J28_GPIO_Port, EN1_Distance_J28_Pin, GPIO_PIN_RESET); }
}

