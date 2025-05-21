#ifndef INC_TOUCH_SENSORS_H_
#define INC_TOUCH_SENSORS_H_
//
#define NO_TOUCH_SENSORS 8

struct TouchSensors_Data_Type {
	int OldSensor[NO_TOUCH_SENSORS];
	int Sensor[NO_TOUCH_SENSORS];
	};

struct TouchSensors_Data_Type *TouchSensors_GetPointer();


void TouchSensors_Init();

void TouchSensors_Update();

int TouchSensor_AnyPressed();

int TouchSensor_AnyChanged();

#endif /* INC_TOUCH_SENSORS_H_ */
