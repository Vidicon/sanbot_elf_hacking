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
#define SCL_Distance_J26_Pin GPIO_PIN_0
#define SCL_Distance_J26_GPIO_Port GPIOC
#define SDA_Distance_J26_Pin GPIO_PIN_1
#define SDA_Distance_J26_GPIO_Port GPIOC
#define EN1_Distance_J26_Pin GPIO_PIN_2
#define EN1_Distance_J26_GPIO_Port GPIOC
#define EN2_Distance_J26_Pin GPIO_PIN_3
#define EN2_Distance_J26_GPIO_Port GPIOC
#define EN3_Distance_J26_Pin GPIO_PIN_1
#define EN3_Distance_J26_GPIO_Port GPIOA
#define EN4_Distance_J26_Pin GPIO_PIN_2
#define EN4_Distance_J26_GPIO_Port GPIOA
#define PF13_Pin GPIO_PIN_13
#define PF13_GPIO_Port GPIOF
#define PF14_Pin GPIO_PIN_14
#define PF14_GPIO_Port GPIOF
#define PF15_Pin GPIO_PIN_15
#define PF15_GPIO_Port GPIOF
#define RightLimitBack_Pin GPIO_PIN_0
#define RightLimitBack_GPIO_Port GPIOG
#define RightLimitUp_Pin GPIO_PIN_1
#define RightLimitUp_GPIO_Port GPIOG
#define PE7_Pin GPIO_PIN_7
#define PE7_GPIO_Port GPIOE
#define PE8_Pin GPIO_PIN_8
#define PE8_GPIO_Port GPIOE
#define PE9_Pin GPIO_PIN_9
#define PE9_GPIO_Port GPIOE
#define RightArmRed_Pin GPIO_PIN_10
#define RightArmRed_GPIO_Port GPIOD
#define RightArmGreen_Pin GPIO_PIN_14
#define RightArmGreen_GPIO_Port GPIOD
#define RightArmBlue_Pin GPIO_PIN_15
#define RightArmBlue_GPIO_Port GPIOD
#define MotionBack_Pin GPIO_PIN_7
#define MotionBack_GPIO_Port GPIOG
#define MotionFront_Pin GPIO_PIN_8
#define MotionFront_GPIO_Port GPIOG
#define LeftArmRed_Pin GPIO_PIN_13
#define LeftArmRed_GPIO_Port GPIOH
#define LeftArmGreen_Pin GPIO_PIN_14
#define LeftArmGreen_GPIO_Port GPIOH
#define LeftArmBlue_Pin GPIO_PIN_15
#define LeftArmBlue_GPIO_Port GPIOH
#define PD6_Pin GPIO_PIN_6
#define PD6_GPIO_Port GPIOD
#define PD7_Pin GPIO_PIN_7
#define PD7_GPIO_Port GPIOD
#define LeftLimitUp_Pin GPIO_PIN_10
#define LeftLimitUp_GPIO_Port GPIOG
#define LeftLimitBack_Pin GPIO_PIN_11
#define LeftLimitBack_GPIO_Port GPIOG
#define BaseRed_Pin GPIO_PIN_12
#define BaseRed_GPIO_Port GPIOG
#define BaseGreen_Pin GPIO_PIN_13
#define BaseGreen_GPIO_Port GPIOG
#define BaseBlue_Pin GPIO_PIN_15
#define BaseBlue_GPIO_Port GPIOG
#define SCL_Distance_J18_Pin GPIO_PIN_8
#define SCL_Distance_J18_GPIO_Port GPIOB
#define SDA_Distance_J18_Pin GPIO_PIN_9
#define SDA_Distance_J18_GPIO_Port GPIOB
#define LeftArmUp_Pin GPIO_PIN_1
#define LeftArmUp_GPIO_Port GPIOE
#define EN1_Distance_J18_Pin GPIO_PIN_4
#define EN1_Distance_J18_GPIO_Port GPIOI
#define EN2_Distance_J18_Pin GPIO_PIN_5
#define EN2_Distance_J18_GPIO_Port GPIOI
#define EN3_Distance_J18_Pin GPIO_PIN_6
#define EN3_Distance_J18_GPIO_Port GPIOI
#define EN4_Distance_J18_Pin GPIO_PIN_7
#define EN4_Distance_J18_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
