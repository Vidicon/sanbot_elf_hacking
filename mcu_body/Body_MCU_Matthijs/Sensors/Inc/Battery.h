/*
 * Battery.h
 *
 *  Created on: Nov 21, 2024
 *      Author: matthijs
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

struct Battery_Sensor_Type {
	uint16_t DeviceType;
	uint16_t FW_Version;
	uint16_t HW_Version;
	uint16_t BatteryState;
	int16_t Temperature;
	int16_t Current;
	int16_t Voltage;
};

struct Battery_Sensor_Type *Battery_GetPointer();

void Battery_Init(I2C_HandleTypeDef *hI2C);

void Battery_Update();



#endif /* INC_BATTERY_H_ */
