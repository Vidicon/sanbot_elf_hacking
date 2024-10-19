/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotGlobals.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
int Update_20Hz;
int Update_16Hz;
int Update_10Hz;
int Update_5Hz;
int Update_2Hz;

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
#define LeftArmBrake_Pin GPIO_PIN_2
#define LeftArmBrake_GPIO_Port GPIOE
#define RightArmUp_Pin GPIO_PIN_3
#define RightArmUp_GPIO_Port GPIOE
#define RightArmBrake_Pin GPIO_PIN_4
#define RightArmBrake_GPIO_Port GPIOE
#define CenterBrake_Pin GPIO_PIN_0
#define CenterBrake_GPIO_Port GPIOF
#define CenterDir_Pin GPIO_PIN_1
#define CenterDir_GPIO_Port GPIOF
#define LeftBrake_Pin GPIO_PIN_2
#define LeftBrake_GPIO_Port GPIOF
#define LeftDir_Pin GPIO_PIN_3
#define LeftDir_GPIO_Port GPIOF
#define RightBrake_Pin GPIO_PIN_4
#define RightBrake_GPIO_Port GPIOF
#define RightDir_Pin GPIO_PIN_5
#define RightDir_GPIO_Port GPIOF
#define RightArmRed_Pin GPIO_PIN_10
#define RightArmRed_GPIO_Port GPIOD
#define RightArmGreen_Pin GPIO_PIN_14
#define RightArmGreen_GPIO_Port GPIOD
#define RightArmBlue_Pin GPIO_PIN_15
#define RightArmBlue_GPIO_Port GPIOD
#define LeftArmRed_Pin GPIO_PIN_13
#define LeftArmRed_GPIO_Port GPIOH
#define LeftArmGreen_Pin GPIO_PIN_14
#define LeftArmGreen_GPIO_Port GPIOH
#define LeftArmBlue_Pin GPIO_PIN_15
#define LeftArmBlue_GPIO_Port GPIOH
#define BaseRed_Pin GPIO_PIN_12
#define BaseRed_GPIO_Port GPIOG
#define BaseGreen_Pin GPIO_PIN_13
#define BaseGreen_GPIO_Port GPIOG
#define BaseBlue_Pin GPIO_PIN_15
#define BaseBlue_GPIO_Port GPIOG
#define LeftArmUp_Pin GPIO_PIN_1
#define LeftArmUp_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
