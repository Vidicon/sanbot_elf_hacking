/*
 * RGBLeds.c
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */
#include "stm32f1xx.h"
#include "RGBLeds_Head.h"
#include <main.h>

static struct RGBLeds_State_Type RGBLeds_State[2];
//static int SelfTestCounterTmo = 0;

void RGBLeds_Init()
{
	RGBLeds_SetColorOff(LeftHead);
	RGBLeds_SetColorOff(RightHead);
}


void RGBLeds_SetAllColors(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Command)
{
	if (Command == LED_On)
	{
		RGBLeds_SetColorOn(BodyPart, Color);
		RGBLeds_BlinkColor(BodyPart, Color, LED_Blink_Off);

	}

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
	if (BodyPart == LeftHead)
	{
		// All off, then set new color
		HAL_GPIO_WritePin(LeftHeadRed_GPIO_Port,   LeftHeadRed_Pin,   GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftHeadGreen_GPIO_Port, LeftHeadGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftHeadBlue_GPIO_Port,  LeftHeadBlue_Pin,  GPIO_PIN_SET);

		if (Color == Red)   {HAL_GPIO_WritePin(LeftHeadRed_GPIO_Port,   LeftHeadRed_Pin,   GPIO_PIN_RESET);}
		if (Color == Green) {HAL_GPIO_WritePin(LeftHeadGreen_GPIO_Port, LeftHeadGreen_Pin, GPIO_PIN_RESET);}
		if (Color == Blue)  {HAL_GPIO_WritePin(LeftHeadBlue_GPIO_Port,  LeftHeadBlue_Pin,  GPIO_PIN_RESET);}
		if (Color == White)
		{
			HAL_GPIO_WritePin(LeftHeadRed_GPIO_Port,   LeftHeadRed_Pin,   GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftHeadGreen_GPIO_Port, LeftHeadGreen_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LeftHeadBlue_GPIO_Port,  LeftHeadBlue_Pin,  GPIO_PIN_RESET);
		}
	}

	if (BodyPart == RightHead)
	{
		// All off, then set new color
		HAL_GPIO_WritePin(RightHeadRed_GPIO_Port,   RightHeadRed_Pin,   GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightHeadGreen_GPIO_Port, RightHeadGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightHeadBlue_GPIO_Port,  RightHeadBlue_Pin,  GPIO_PIN_SET);

		if (Color == Red)   {HAL_GPIO_WritePin(RightHeadRed_GPIO_Port,   RightHeadRed_Pin,   GPIO_PIN_RESET);}
		if (Color == Green) {HAL_GPIO_WritePin(RightHeadGreen_GPIO_Port, RightHeadGreen_Pin, GPIO_PIN_RESET);}
		if (Color == Blue)  {HAL_GPIO_WritePin(RightHeadBlue_GPIO_Port,  RightHeadBlue_Pin,  GPIO_PIN_RESET);}
		if (Color == White)
		{
			HAL_GPIO_WritePin(RightHeadRed_GPIO_Port,   RightHeadRed_Pin,   GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightHeadGreen_GPIO_Port, RightHeadGreen_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RightHeadBlue_GPIO_Port,  RightHeadBlue_Pin,  GPIO_PIN_RESET);
		}
	}
}

void RGBLeds_SetColorOff(enum ENUM_BodyParts BodyPart)
{
	if (BodyPart == LeftHead)
	{
		HAL_GPIO_WritePin(LeftHeadRed_GPIO_Port, 	LeftHeadRed_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftHeadGreen_GPIO_Port, 	LeftHeadGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LeftHeadBlue_GPIO_Port, 	LeftHeadBlue_Pin, GPIO_PIN_SET);
	}

	if (BodyPart == RightHead)
	{
		HAL_GPIO_WritePin(RightHeadRed_GPIO_Port, 	RightHeadRed_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightHeadGreen_GPIO_Port,  RightHeadGreen_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RightHeadBlue_GPIO_Port, 	RightHeadBlue_Pin, GPIO_PIN_SET);
	}
}

void RGBLeds_BlinkColor(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Blink)
{
	RGBLeds_State[BodyPart - LeftHead].Color   = Color;
	RGBLeds_State[BodyPart - LeftHead].Blink   = Blink;
	RGBLeds_State[BodyPart - LeftHead].Counter = 0;
}

void RGBLeds_Update10Hz()
{
	for (int i = 0; i < 2; i++)
	{
		if (RGBLeds_State[i].Blink == LED_Blink_Slow)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i+LeftHead), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 5) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i+LeftHead));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 10) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_Fast)
		{
			// Special XMAS version
			if (RGBLeds_State[i].Color == RedGreen)
			{
				if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i+LeftHead), Green);}
				if (RGBLeds_State[i].Counter == 3) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i+LeftHead), Red);}
			}
			else
			{
				if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i+LeftHead), RGBLeds_State[i].Color);}
				if (RGBLeds_State[i].Counter == 3) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i+LeftHead));}
			}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 6) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_VeryFast)
		{
			if (RGBLeds_State[i].Counter == 0) { RGBLeds_SetColorOn ((enum ENUM_BodyParts)(i+LeftHead), RGBLeds_State[i].Color);}
			if (RGBLeds_State[i].Counter == 2) { RGBLeds_SetColorOff((enum ENUM_BodyParts)(i+LeftHead));}

			RGBLeds_State[i].Counter += 1;
			if (RGBLeds_State[i].Counter >= 4) {RGBLeds_State[i].Counter = 0;}
		}

		if (RGBLeds_State[i].Blink == LED_Blink_Off)
		{
//			RGBLeds_SetColorOff((enum ENUM_BodyParts)(i));
		}
	}
}

void RGBLeds_SelfTest(enum ENUM_Booleans Enabled)
{
	if (Enabled == True)
	{
		RGBLeds_BlinkColor(LeftHead, Red, LED_Blink_Slow);
		RGBLeds_BlinkColor(RightHead, Green, LED_Blink_Fast);
	}
	else
	{
		RGBLeds_SetColorOff(LeftHead);
		RGBLeds_BlinkColor(LeftHead, None, LED_Blink_Off);

		RGBLeds_SetColorOff(RightHead);
		RGBLeds_BlinkColor(RightHead, None, LED_Blink_Off);
	}
}


