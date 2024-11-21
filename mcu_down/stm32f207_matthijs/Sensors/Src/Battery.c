/*
 * Battery.c
 *
 *  Created on: Nov 21, 2024
 *      Author: matthijs
 */
#include "stm32f2xx_hal.h"
#include <string.h>
#include "protocol_0x55.h"
#include "Battery.h"


struct Battery_Sensor_Type BatteryData;

I2C_HandleTypeDef *Battery_I2C;
uint8_t Battery_Address = 0x16;
HAL_StatusTypeDef status;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct Battery_Sensor_Type *Battery_GetPointer()
{
	return &BatteryData;
}

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
void Battery_Init(I2C_HandleTypeDef *hI2C)
{
	Battery_I2C = hI2C;
}

void Battery_Update()
{


}
