/*
 * Battery.h
 *
 *  Created on: Nov 21, 2024
 *      Author: matthijs
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

struct Battery_Sensor_Type {
	uint8_t Raw[6];
	int16_t X;
	int16_t Y;
	int16_t Z;
};

struct Battery_Sensor_Type *Battery_GetPointer();

void Battery_Init(I2C_HandleTypeDef *hI2C);

void Battery_Update();



#endif /* INC_BATTERY_H_ */
