#include "Base.h"
#include "stm32f2xx.h"
#include <main.h>

struct Base_State_Type Motor1_State;
struct Base_State_Type Motor1_State;

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void Base_Init(TIM_HandleTypeDef *htim)
{
//	LeftArm_State.Angle = 0;
//	LeftArm_State.Direction = Arm_Up;
//	LeftArm_State.Homed = NotHomed;
//	LeftArm_State.MotionState = Motion_Disabled;
//	LeftArm_State.Timer = 0;
//	LeftArm_State.SelTestRunning = 0;
//	LeftArm_State.TIM = htim;
//	LeftArm_State.Speed = 0;


//	HAL_TIM_Base_Start(htim);
//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void Base_SelfTest()
{
//	// Force a home sequence
//	LeftArm_EnableBrake(False);
//
//	LeftArm_State.SelTestRunning = 1;
//	LeftArm_State.MotionState = Motion_MovingUp;
//	LeftArm_State.Speed = 10;
}
