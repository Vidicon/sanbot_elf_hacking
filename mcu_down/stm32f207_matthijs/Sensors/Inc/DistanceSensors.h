/*
 * DistanceSensors.h
 *
 *  Created on: Nov 1, 2024
 *      Author: matthijs
 */

#ifndef INC_DISTANCESENSORS_H_
#define INC_DISTANCESENSORS_H_

struct Distance_Sensor_Type {
	int Distance[4];
	int SelectedSensor;
};

struct Distance_Sensor_Type *DistanceSensors_GetPointer();

void DistanceSensors_Init();

void DistanceSensors_Update10Hz();

void DistanceSensors_Select(int SensorID);

#endif /* INC_DISTANCESENSORS_H_ */
