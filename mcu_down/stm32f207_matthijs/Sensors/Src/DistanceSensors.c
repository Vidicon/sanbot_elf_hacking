/*
 * Distance_Sensors.c
 *
 *  Created on: Nov 1, 2024
 *      Author: matthijs
 */

#include "DistanceSensors.h"

int SelectedSensor = 0;

//------------------------------------------------------------------------------
// Functions to call from outside this file
//------------------------------------------------------------------------------
void DistanceSensors_Init()
{
	SelectedSensor = 0;
	DistanceSensors_Select(SelectedSensor);
}

void DistanceSensors_Update10Hz()
{



	// Last step is to select the next sensor
	SelectedSensor += 1;
	if (SelectedSensor >= 4) { SelectedSensor = 0;}

	DistanceSensors_Select(SelectedSensor);
}


void DistanceSensors_Select(int SensorID)
{

}

