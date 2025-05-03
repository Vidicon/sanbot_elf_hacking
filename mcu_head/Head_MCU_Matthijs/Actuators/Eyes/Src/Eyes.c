
#include "stm32f1xx.h"
#include <main.h>
#include <stdio.h>
#include <string.h>

#include "Eyes.h"
#include "SSD1305_eyes.h"
#include "RobotGlobals.h"
#include "SSD1305_eyes.h"
#include "default_eyes.h"
#include "fire_eyes.h"
#include "hearts.h"
#include "tears.h"

static int Selected_Left_Eye = 1;
static int Selected_Right_Eye = 1;

static OLED_HandleTypeDef *left_eye;
static OLED_HandleTypeDef *right_eye;

void Eyes_Init(OLED_HandleTypeDef *left_eye_in, OLED_HandleTypeDef *right_eye_in)
{
	left_eye = left_eye_in;
	right_eye = right_eye_in;
}

void Eyes_Select(char Byte1, char Byte2)
{
	Selected_Left_Eye = Byte1;
	Selected_Right_Eye = Byte2;

	Update_Eyes(False);
}

void Update_Eyes(int BlinkEye)
{
	if (Selected_Left_Eye == 0)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(left_eye, &nobleo_logo);
		}
		else
		{
			SSD1305_writeDisplay(left_eye, &nobleo_logo);
		}
	}

	if (Selected_Left_Eye == 1)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(left_eye, &default_left_eye_closed);
		}
		else
		{
			SSD1305_writeDisplay(left_eye, &default_left_eye_open);
		}
	}

	if (Selected_Left_Eye == 2)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(left_eye, &fire_left_eye_low);
		}
		else
		{
			SSD1305_writeDisplay(left_eye, &fire_left_eye_high);
		}
	}

	if (Selected_Left_Eye == 3)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(left_eye, &tears_left_high);
		}
		else
		{
			SSD1305_writeDisplay(left_eye, &tears_left_low);
		}
	}

	if (Selected_Left_Eye == 4)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(left_eye, &heart_left_high);
		}
		else
		{
			SSD1305_writeDisplay(left_eye, &heart_left_low);
		}
	}

	//-----------------------------------------------------------------------------

	if (Selected_Right_Eye == 0)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(right_eye, &sara_logo);
		}
		else
		{
			SSD1305_writeDisplay(right_eye, &sara_logo);
		}
	}

	if (Selected_Right_Eye == 1)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(right_eye, &default_right_eye_closed);
		}
		else
		{
			SSD1305_writeDisplay(right_eye, &default_right_eye_open);
		}
	}

	if (Selected_Right_Eye == 2)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(right_eye, &fire_right_eye_low);
		}
		else
		{
			SSD1305_writeDisplay(right_eye, &fire_right_eye_high);
		}
	}

	if (Selected_Right_Eye == 3)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(right_eye, &tears_right_high);
		}
		else
		{
			SSD1305_writeDisplay(right_eye, &tears_right_low);
		}
	}

	if (Selected_Right_Eye == 4)
	{
		if (BlinkEye == True)
		{
			SSD1305_writeDisplay(right_eye, &heart_right_high);
		}
		else
		{
			SSD1305_writeDisplay(right_eye, &heart_right_low);
		}
	}
}
