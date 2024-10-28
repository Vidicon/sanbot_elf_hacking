#include "MotionSensors.h"
#include "protocol_0x55.h"
#include <main.h>

struct MotionSensors_Data_Type MotionSensors_State;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct MotionSensors_Data_Type *MotionSensors_GetPointer()
{
	return &MotionSensors_State;
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void MotionSensors_Init()
{
	// Read actual sensor values
	// TO DO
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void MotionSensors_Update10Hz()
{
	for (int i=0; i<2; i++)
	{
		if (i == 0) {MotionSensors_State.CurrentValue[i] = HAL_GPIO_ReadPin(MotionFront_GPIO_Port, MotionFront_Pin);}
		if (i == 1) {MotionSensors_State.CurrentValue[i] = HAL_GPIO_ReadPin(MotionBack_GPIO_Port, MotionBack_Pin);}

		if (MotionSensors_State.CurrentValue[i] != MotionSensors_State.PreviousValue[i])
		{
			SendMotionSensors(MotionSensors_GetPointer());
		}

		// Store new state
		MotionSensors_State.PreviousValue[i] = MotionSensors_State.CurrentValue[i];
	}
}

