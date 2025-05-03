/*
 * Eyes.h
 *
 *  Created on: May 3, 2025
 *      Author: matthijs
 */

#ifndef EYES_INC_EYES_H_
#define EYES_INC_EYES_H_

#include "SSD1305_eyes.h"

void Eyes_Init(OLED_HandleTypeDef *left_eye, OLED_HandleTypeDef *right_eye);

void Eyes_Select(char Byte1, char Byte2);

void Update_Eyes(int BlinkEye);


#endif /* EYES_INC_EYES_H_ */
