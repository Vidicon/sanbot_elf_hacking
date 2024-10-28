#include "MotionSensors.h"
#include "protocol_0x55.h"

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
		if (MotionSensors_State.CurrentValue[i] != MotionSensors_State.PreviousValue[i])
		{
			SendMotionSensors(MotionSensors_GetPointer());
		}

		// Store new state
		MotionSensors_State.PreviousValue[i] = MotionSensors_State.CurrentValue[i];
	}
}

