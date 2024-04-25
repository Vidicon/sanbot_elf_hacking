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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WingRightRed_Pin GPIO_PIN_10
#define WingRightRed_GPIO_Port GPIOD
#define WingRightGreen_Pin GPIO_PIN_14
#define WingRightGreen_GPIO_Port GPIOD
#define WingRightBlue_Pin GPIO_PIN_15
#define WingRightBlue_GPIO_Port GPIOD
#define WingLeftRed_Pin GPIO_PIN_13
#define WingLeftRed_GPIO_Port GPIOH
#define WingLeftGreen_Pin GPIO_PIN_14
#define WingLeftGreen_GPIO_Port GPIOH
#define WingLeftBlue_Pin GPIO_PIN_15
#define WingLeftBlue_GPIO_Port GPIOH
#define BottomRed_Pin GPIO_PIN_12
#define BottomRed_GPIO_Port GPIOG
#define BottomGreen_Pin GPIO_PIN_13
#define BottomGreen_GPIO_Port GPIOG
#define BottomBlue_Pin GPIO_PIN_15
#define BottomBlue_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
