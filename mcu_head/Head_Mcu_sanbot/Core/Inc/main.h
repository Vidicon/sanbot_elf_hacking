/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
void getdata(uint8_t * rxBuffer, uint32_t size);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOUCH_92_Pin GPIO_PIN_2
#define TOUCH_92_GPIO_Port GPIOE
#define TOUCH_84_Pin GPIO_PIN_3
#define TOUCH_84_GPIO_Port GPIOE
#define TOUCH_86_Pin GPIO_PIN_4
#define TOUCH_86_GPIO_Port GPIOE
#define TOUCH_91_Pin GPIO_PIN_5
#define TOUCH_91_GPIO_Port GPIOE
#define TOUCH_NC_Pin GPIO_PIN_6
#define TOUCH_NC_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define PAN1_SENS_Pin GPIO_PIN_2
#define PAN1_SENS_GPIO_Port GPIOF
#define PAN2_SENS_Pin GPIO_PIN_3
#define PAN2_SENS_GPIO_Port GPIOF
#define TIL1_SENS_Pin GPIO_PIN_4
#define TIL1_SENS_GPIO_Port GPIOF
#define TIL2_SENS_Pin GPIO_PIN_5
#define TIL2_SENS_GPIO_Port GPIOF
#define TOP_LED_Pin GPIO_PIN_9
#define TOP_LED_GPIO_Port GPIOF
#define PAD_POWER_EN_Pin GPIO_PIN_10
#define PAD_POWER_EN_GPIO_Port GPIOF
#define HEAD_LED_EN_Pin GPIO_PIN_3
#define HEAD_LED_EN_GPIO_Port GPIOC
#define TOP_BUTTON_Pin GPIO_PIN_0
#define TOP_BUTTON_GPIO_Port GPIOA
#define HEAD_LED_PWM_Pin GPIO_PIN_1
#define HEAD_LED_PWM_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define OneWIRE_DS18820_Pin GPIO_PIN_4
#define OneWIRE_DS18820_GPIO_Port GPIOC
#define CAM_3D_POW_EN_Pin GPIO_PIN_0
#define CAM_3D_POW_EN_GPIO_Port GPIOG
#define USB_EN_Pin GPIO_PIN_1
#define USB_EN_GPIO_Port GPIOG
#define LEFT_LED_R_Pin GPIO_PIN_8
#define LEFT_LED_R_GPIO_Port GPIOE
#define LEFT_LED_G_Pin GPIO_PIN_10
#define LEFT_LED_G_GPIO_Port GPIOE
#define LEFT_LED_B_Pin GPIO_PIN_12
#define LEFT_LED_B_GPIO_Port GPIOE
#define TIL_EN_Pin GPIO_PIN_14
#define TIL_EN_GPIO_Port GPIOE
#define PAN_EN_Pin GPIO_PIN_15
#define PAN_EN_GPIO_Port GPIOE
#define OLED_L_CS_Pin GPIO_PIN_14
#define OLED_L_CS_GPIO_Port GPIOB
#define TIL_DIR_Pin GPIO_PIN_11
#define TIL_DIR_GPIO_Port GPIOD
#define PAN_DIR_Pin GPIO_PIN_12
#define PAN_DIR_GPIO_Port GPIOD
#define RIGHT_LED_R_Pin GPIO_PIN_13
#define RIGHT_LED_R_GPIO_Port GPIOD
#define RIGHT_LED_G_Pin GPIO_PIN_14
#define RIGHT_LED_G_GPIO_Port GPIOD
#define RIGHT_LED_B_Pin GPIO_PIN_15
#define RIGHT_LED_B_GPIO_Port GPIOD
#define OLED_R_CS_Pin GPIO_PIN_15
#define OLED_R_CS_GPIO_Port GPIOA
#define OLED_RESET_Pin GPIO_PIN_5
#define OLED_RESET_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOD
#define TOUCH_83_Pin GPIO_PIN_15
#define TOUCH_83_GPIO_Port GPIOG
#define TOUCH_85_Pin GPIO_PIN_0
#define TOUCH_85_GPIO_Port GPIOE
#define TOUCH_90_Pin GPIO_PIN_1
#define TOUCH_90_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
