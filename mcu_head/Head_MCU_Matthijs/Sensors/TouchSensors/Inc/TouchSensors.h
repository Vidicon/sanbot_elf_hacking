#ifndef INC_TOUCH_SENSORS_H_
#define INC_TOUCH_SENSORS_H_
//

#endif /* INC_TOUCH_SENSORS_H_ */

struct TouchSensors_Data_Type {
	int Sensor[8];
	};

//------------------------------------------------
struct TouchSensors_Data_Type *TouchSensor_GetPointer();

void TouchSensors_Init();

void TouchSensors_Update();
