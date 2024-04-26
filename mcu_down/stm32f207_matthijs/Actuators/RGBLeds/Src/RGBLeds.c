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

void RGBLeds_SetColorOn(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color)
{
	if (BodyPart == LeftArm)
	{
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

void RGBLeds_BlinkColor(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Blink Blink)
{
	RGBLeds_State[BodyPart].Color = Color;
	RGBLeds_State[BodyPart].Blink = Blink;
	RGBLeds_State[BodyPart].Counter = 0;
}

void RGBLeds_Update10Hz()
{
	for (int i = 0; i < 3; i++)
	{
		if (RGBLeds_State[i].Blink == Blink_Slow)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 5) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 10) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == Blink_Fast)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 3) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 6) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == Blink_VeryFast)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 2) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 4) {RGBLeds_State[i].Counter = 0;}
		}
	}

	//---------------------------------------------------------
	// Selftest update
	//---------------------------------------------------------
	SelfTestCounterTmo += 1;

	if (SelfTestCounterTmo >= 5 * UPDATE_10HZ)
	{
		RGBLeds_SelfTest(False);

		RGBLeds_SetColorOn(LeftArm, White);
		RGBLeds_SetColorOn(RightArm, White);
	}
}

void RGBLeds_SelfTest(enum ENUM_Booleans Enabled)
{
	if (Enabled == True)
	{
		SelfTestCounterTmo = 0;

		RGBLeds_BlinkColor(LeftArm, Red, Blink_Slow);
		RGBLeds_BlinkColor(RightArm, Green, Blink_Fast);
		RGBLeds_BlinkColor(Base, White, Blink_VeryFast);
	}
	else
	{
		RGBLeds_BlinkColor(LeftArm, Red, Blink_Off);
		RGBLeds_SetColorOff(LeftArm);

		RGBLeds_BlinkColor(RightArm, Green, Blink_Off);
		RGBLeds_SetColorOff(RightArm);

		RGBLeds_BlinkColor(Base, White, Blink_Off);
		RGBLeds_SetColorOff(Base);
	}
}


