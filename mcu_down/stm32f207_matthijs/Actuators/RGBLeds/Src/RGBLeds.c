/*
 * RGBLeds.c
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */
#include "stm32f2xx.h"
#include "RGBLeds.h"
#include <main.h>

struct RGBLeds_State_Type RGBLeds_State[3];
int SelfTestCounterTmo = 0;

void RGBLeds_Init()
{
	RGBLeds_SetColorOff(LeftArm);
	RGBLeds_SetColorOff(RightArm);
	RGBLeds_SetColorOff(Base);
}


void RGBLeds_SetAllColors(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Command)
{
	if (Command == LED_On) 	{ RGBLeds_SetColorOn(BodyPart, Color);}
	if (Command == LED_Off)
	{
		RGBLeds_SetColorOff(BodyPart);
		RGBLeds_BlinkColor(BodyPart, Color, LED_Blink_Off);
	}


	if ((Command >= LED_Blink_Off) && (Command <= LED_Blink_VeryFast))
	{
		RGBLeds_BlinkColor(BodyPart, Color, Command);
	}
}

void RGBLeds_SetColorOn(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color)
{
	if (BodyPart == LeftArm)
	{
		// All off, then set new color
		HAL_GPIO_WritePin(LeftArmRed_GPIO_Port,   LeftArmRed_Pin,   GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftArmGreen_GPIO_Port, LeftArmGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftArmBlue_GPIO_Port,  LeftArmBlue_Pin,  GPIO_PIN_SET);

		if (Color == Red)   {HAL_GPIO_WritePin(LeftArmRed_GPIO_Port,   LeftArmRed_Pin,   GPIO_PIN_RESET);}
		if (Color == Green) {HAL_GPIO_WritePin(LeftArmGreen_GPIO_Port, LeftArmGreen_Pin, GPIO_PIN_RESET);}
		if (Color == Blue)  {HAL_GPIO_WritePin(LeftArmBlue_GPIO_Port,  LeftArmBlue_Pin,  GPIO_PIN_RESET);}
		if (Color == White)
		{
			HAL_GPIO_WritePin(LeftArmRed_GPIO_Port,   LeftArmRed_Pin,   GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftArmGreen_GPIO_Port, LeftArmGreen_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftArmBlue_GPIO_Port,  LeftArmBlue_Pin,  GPIO_PIN_RESET);
		}
	}

	if (BodyPart == RightArm)
	{
		// All off, then set new color
		HAL_GPIO_WritePin(RightArmRed_GPIO_Port,   RightArmRed_Pin,   GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightArmGreen_GPIO_Port, RightArmGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightArmBlue_GPIO_Port,  RightArmBlue_Pin,  GPIO_PIN_SET);

		if (Color == Red)   {HAL_GPIO_WritePin(RightArmRed_GPIO_Port,   RightArmRed_Pin,   GPIO_PIN_RESET);}
		if (Color == Green) {HAL_GPIO_WritePin(RightArmGreen_GPIO_Port, RightArmGreen_Pin, GPIO_PIN_RESET);}
		if (Color == Blue)  {HAL_GPIO_WritePin(RightArmBlue_GPIO_Port,  RightArmBlue_Pin,  GPIO_PIN_RESET);}
		if (Color == White)
		{
			HAL_GPIO_WritePin(RightArmRed_GPIO_Port,   RightArmRed_Pin,   GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightArmGreen_GPIO_Port, RightArmGreen_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightArmBlue_GPIO_Port,  RightArmBlue_Pin,  GPIO_PIN_RESET);
		}
	}

	if (BodyPart == Base)
	{
		// All off, then set new color
		HAL_GPIO_WritePin(BaseRed_GPIO_Port,   	BaseRed_Pin,   GPIO_PIN_SET);
		HAL_GPIO_WritePin(BaseGreen_GPIO_Port, 	BaseGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BaseBlue_GPIO_Port,  	BaseBlue_Pin,  GPIO_PIN_SET);

		if (Color == Red)   {HAL_GPIO_WritePin(BaseRed_GPIO_Port,   	BaseRed_Pin,   GPIO_PIN_RESET);}
		if (Color == Green) {HAL_GPIO_WritePin(BaseGreen_GPIO_Port, 	BaseGreen_Pin, GPIO_PIN_RESET);}
		if (Color == Blue)  {HAL_GPIO_WritePin(BaseBlue_GPIO_Port,  	BaseBlue_Pin,  GPIO_PIN_RESET);}
		if (Color == White)
		{
			HAL_GPIO_WritePin(BaseRed_GPIO_Port,   	BaseRed_Pin,   GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BaseGreen_GPIO_Port, 	BaseGreen_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BaseBlue_GPIO_Port,  	BaseBlue_Pin,  GPIO_PIN_RESET);
		}
	}
}

void RGBLeds_SetColorOff(enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftArm)
	{
		HAL_GPIO_WritePin(LeftArmRed_GPIO_Port, 	LeftArmRed_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftArmGreen_GPIO_Port, 	LeftArmGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftArmBlue_GPIO_Port, 	LeftArmBlue_Pin, GPIO_PIN_SET);
	}

	if (BodyPart == RightArm)
	{
		HAL_GPIO_WritePin(RightArmRed_GPIO_Port, 	RightArmRed_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightArmGreen_GPIO_Port,  RightArmGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightArmBlue_GPIO_Port, 	RightArmBlue_Pin, GPIO_PIN_SET);
	}

	if (BodyPart == Base)
	{
		HAL_GPIO_WritePin(BaseRed_GPIO_Port, 		BaseRed_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BaseGreen_GPIO_Port, 		BaseGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BaseBlue_GPIO_Port, 		BaseBlue_Pin, GPIO_PIN_SET);
	}
}

void RGBLeds_BlinkColor(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Blink)
{
	RGBLeds_State[BodyPart].Color   = Color;
	RGBLeds_State[BodyPart].Blink   = Blink;
	RGBLeds_State[BodyPart].Counter = 0;
}

void RGBLeds_Update10Hz()
{
	for (int i = 0; i < 3; i++)
	{
		if (RGBLeds_State[i].Blink == LED_Blink_Slow)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 5) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 10) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_Fast)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 3) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 6) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_VeryFast)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 2) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 4) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_Off)
		{
//			RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));
		}
	}

	//---------------------------------------------------------
	// Selftest update
	//---------------------------------------------------------

	if (SelfTestCounterTmo > 0)
	{
		if (SelfTestCounterTmo == 1) { RGBLeds_SelfTest(False);}

		SelfTestCounterTmo -= 1;
	}

}

void RGBLeds_SelfTest(enum ENUM_Booleans Enabled)
{
	if (Enabled == True)
	{
		SelfTestCounterTmo = 5 * UPDATE_10HZ;

		RGBLeds_BlinkColor(LeftArm, Red, LED_Blink_Slow);
		RGBLeds_BlinkColor(RightArm, Green, LED_Blink_Fast);
		RGBLeds_BlinkColor(Base, White, LED_Blink_VeryFast);
	}
	else
	{
		RGBLeds_SetColorOff(LeftArm);
		RGBLeds_BlinkColor(LeftArm, None, LED_Blink_Off);

		RGBLeds_SetColorOff(RightArm);
		RGBLeds_BlinkColor(RightArm, None, LED_Blink_Off);

		RGBLeds_SetColorOff(Base);
		RGBLeds_BlinkColor(Base, None, LED_Blink_Off);
	}
}


