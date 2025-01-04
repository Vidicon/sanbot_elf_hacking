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
#include <Cliff_Soft_I2C.h>
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

void DistanceSensors_Update()
{
	// Left: 4 sensors 0, 1, 2, 3
	if (DistanceData.SelectedSensor <= 3)
	{

		Left_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Left_Soft_I2C_Read(0x40);
	}
	// Right: 4 sensors 4, 5, 6, 7
	else if (DistanceData.SelectedSensor <= 7)
	{
		Right_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Right_Soft_I2C_Read(0x40);
	}
	// Center: 3 sensors 8, 9, 10
	else if (DistanceData.SelectedSensor <= 10)
	{
		Mid_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Mid_Soft_I2C_Read(0x40);
	}
	// Cliff: 2 sensors 11, 12
	else
	{
		Cliff_Soft_I2C_Write(0x40, 0x5E);
		DistanceData.Distance[DistanceData.SelectedSensor] = Cliff_Soft_I2C_Read(0x40);
	}

	// Last step is to select the next sensor
	// Total = 13 sensors
	DistanceData.SelectedSensor += 1;
	if (DistanceData.SelectedSensor >= NO_DISTANCE_SENSORS)
	{
		DistanceData.SelectedSensor = 0;

		// Send data to host
		SendDistanceSensors(DistanceSensors_GetPointer());
	}

	// Select next sensor in advance by setting GPIO high
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

	//-----------------------------------------------------------------------------------------------
	// 2 Cliff sensors
	//-----------------------------------------------------------------------------------------------
	if (SensorID == 11) { HAL_GPIO_WritePin(EN1_Distance_J22_GPIO_Port, EN1_Distance_J22_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN1_Distance_J22_GPIO_Port, EN1_Distance_J22_Pin, GPIO_PIN_RESET); }

	if (SensorID == 12) { HAL_GPIO_WritePin(EN2_Distance_J22_GPIO_Port, EN2_Distance_J22_Pin, GPIO_PIN_SET);}
	else { HAL_GPIO_WritePin(EN2_Distance_J22_GPIO_Port, EN2_Distance_J22_Pin, GPIO_PIN_RESET); }
}

