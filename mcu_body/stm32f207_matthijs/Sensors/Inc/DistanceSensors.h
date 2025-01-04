/*
 * DistanceSensors.h
 *
 *  Created on: Nov 1, 2024
 *      Author: matthijs
 */

#ifndef INC_DISTANCESENSORS_H_
#define INC_DISTANCESENSORS_H_

#define NO_DISTANCE_SENSORS 13

struct Distance_Sensor_Type {
	int Distance[NO_DISTANCE_SENSORS];
	int SelectedSensor;
};

struct Distance_Sensor_Type *DistanceSensors_GetPointer();

void DistanceSensors_Init();

void DistanceSensors_Update();

void DistanceSensors_Select(int SensorID);

#endif /* INC_DISTANCESENSORS_H_ */
