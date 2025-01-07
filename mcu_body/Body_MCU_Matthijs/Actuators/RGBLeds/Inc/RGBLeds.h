/*
 * RGBLeds.h
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */

#ifndef RGBLEDS_INC_RGBLEDS_H_
#define RGBLEDS_INC_RGBLEDS_H_

#include "RobotGlobals.h"

enum ENUM_RGBLeds_Color {
    None,
	Red,
    Green,
    Blue,
	White,
	RedGreen
};

enum ENUM_RGBLeds_Command {
	LED_NoCommand,
	LED_Off,
	LED_On,
	LED_Blink_Off,
	LED_Blink_Slow,
	LED_Blink_Fast,
	LED_Blink_VeryFast
};

// Declare a variable of type Color
extern enum ENUM_RGBLeds_Color RGBLeds_Color;

struct RGBLeds_State_Type {
	enum ENUM_RGBLeds_Color Color;
	enum ENUM_RGBLeds_Command Blink;
	int Counter;
	};


void RGBLeds_Init();

void RGBLeds_SetAllColors(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Command);

void RGBLeds_SetColorOn(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color);

void RGBLeds_SetColorOff(enum ENUM_BodyParts BodyPart);

void RGBLeds_BlinkColor(enum ENUM_BodyParts BodyPart, enum ENUM_RGBLeds_Color Color, enum ENUM_RGBLeds_Command Blink);

void RGBLeds_Update10Hz();

void RGBLeds_SelfTest(enum ENUM_Booleans Enabled);


#endif /* RGBLEDS_INC_RGBLEDS_H_ */
