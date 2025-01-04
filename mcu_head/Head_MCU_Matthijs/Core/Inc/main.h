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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
int Update_25Hz;
int Update_20Hz;
int Update_16Hz;
int Update_10Hz;
int Update_5Hz;
int Update_2Hz;
int Update_1Hz;
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
#define USB_ENABLE_LOW_Pin GPIO_PIN_1
#define USB_ENABLE_LOW_GPIO_Port GPIOG
#define LeftHeadRed_Pin GPIO_PIN_8
#define LeftHeadRed_GPIO_Port GPIOE
#define LeftHeadGreen_Pin GPIO_PIN_10
#define LeftHeadGreen_GPIO_Port GPIOE
#define LeftHeadBlue_Pin GPIO_PIN_12
#define LeftHeadBlue_GPIO_Port GPIOE
#define RightHeadRed_Pin GPIO_PIN_13
#define RightHeadRed_GPIO_Port GPIOD
#define RightHeadGreen_Pin GPIO_PIN_14
#define RightHeadGreen_GPIO_Port GPIOD
#define RightHeadBlue_Pin GPIO_PIN_15
#define RightHeadBlue_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
