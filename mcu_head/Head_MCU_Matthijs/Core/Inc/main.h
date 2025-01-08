/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define DEMO

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PanPosSensor_Pin GPIO_PIN_2
#define PanPosSensor_GPIO_Port GPIOF
#define PanNegSensor_Pin GPIO_PIN_3
#define PanNegSensor_GPIO_Port GPIOF
#define TiltPosSensor_Pin GPIO_PIN_4
#define TiltPosSensor_GPIO_Port GPIOF
#define TiltNegSensor_Pin GPIO_PIN_5
#define TiltNegSensor_GPIO_Port GPIOF
#define HeadLedEnable_Pin GPIO_PIN_3
#define HeadLedEnable_GPIO_Port GPIOC
#define HeadTopButton_Pin GPIO_PIN_0
#define HeadTopButton_GPIO_Port GPIOA
#define USB_ENABLE_LOW_Pin GPIO_PIN_1
#define USB_ENABLE_LOW_GPIO_Port GPIOG
#define LeftHeadRed_Pin GPIO_PIN_8
#define LeftHeadRed_GPIO_Port GPIOE
#define LeftHeadGreen_Pin GPIO_PIN_10
#define LeftHeadGreen_GPIO_Port GPIOE
#define LeftHeadBlue_Pin GPIO_PIN_12
#define LeftHeadBlue_GPIO_Port GPIOE
#define TiltEnable_Pin GPIO_PIN_14
#define TiltEnable_GPIO_Port GPIOE
#define PanEnable_Pin GPIO_PIN_15
#define PanEnable_GPIO_Port GPIOE
#define OLED_L_CS_Pin GPIO_PIN_14
#define OLED_L_CS_GPIO_Port GPIOB
#define TiltDirection_Pin GPIO_PIN_11
#define TiltDirection_GPIO_Port GPIOD
#define PanDirection_Pin GPIO_PIN_12
#define PanDirection_GPIO_Port GPIOD
#define RightHeadRed_Pin GPIO_PIN_13
#define RightHeadRed_GPIO_Port GPIOD
#define RightHeadGreen_Pin GPIO_PIN_14
#define RightHeadGreen_GPIO_Port GPIOD
#define RightHeadBlue_Pin GPIO_PIN_15
#define RightHeadBlue_GPIO_Port GPIOD
#define OLED_R_CS_Pin GPIO_PIN_15
#define OLED_R_CS_GPIO_Port GPIOA
#define OLED_RESET_Pin GPIO_PIN_5
#define OLED_RESET_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

//#define TOP_BUTTON_Pin GPIO_PIN_0
//#define TOP_BUTTON_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
