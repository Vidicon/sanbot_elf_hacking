#include "Arms.h"
#include "stm32f2xx.h"
#include <main.h>

struct Arm_State_Type LeftArm_State;
struct Arm_State_Type RightArm_State;

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void LeftArm_Init()
{
	LeftArm_State.Angle = 0;
	LeftArm_State.Direction = Arm_Up;
	LeftArm_State.Homed = NotHomed;
	LeftArm_State.MotionState = Motion_Disabled;
}

void LeftArm_SelfTest()
{
	// Force a home sequence
}

void LeftArm_MoveToAngle(int TargetAngle)
{
	// Move if correctly homed
}

void LeftArm_Update10Hz()
{

}


//----------------------------------------------------------------
//
//----------------------------------------------------------------
void RightArm_Init()
{
	RightArm_State.Angle = 0;
	RightArm_State.Direction = Arm_Up;
	RightArm_State.Homed = NotHomed;
	RightArm_State.MotionState = Motion_Disabled;
}

void RightArm_SelfTest()
{
	GenericArm_Init(RightArm_State);
}

void RightArm_MoveToAngle(int TargetAngle)
{

}

void RightArm_Update10Hz()
{

}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void GenericArm_Init(struct Arm_State_Type LeftArm_State)
{

}

