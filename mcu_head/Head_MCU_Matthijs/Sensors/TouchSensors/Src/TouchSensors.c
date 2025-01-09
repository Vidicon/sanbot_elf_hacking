/*
 * RGBLeds.c
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */
#include "TouchSensors.h"

struct TouchSensors_Data_Type TouchSensorData;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct TouchSensors_Data_Type *TouchSensor_GetPointer()
{
	return &TouchSensorData;
}


void TouchSensors_Init()
{
}

void TouchSensors_Update()
{

}
