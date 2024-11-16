/*
 * Compass.h
 *
 *  Created on: Nov 13, 2024
 *      Author: matthijs
 */

#ifndef INC_COMPASS_H_
#define INC_COMPASS_H_

struct Compass_Sensor_Type {
	uint8_t Raw[6];
	int16_t X;
	int16_t Y;
	int16_t Z;
};

struct Compass_Sensor_Type *Compass_GetPointer();

void Compass_Init(I2C_HandleTypeDef *hI2C);

void Compass_Update();


#endif /* INC_COMPASS_H_ */
