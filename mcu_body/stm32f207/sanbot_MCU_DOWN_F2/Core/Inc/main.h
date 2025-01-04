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
#include "stm32f2xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MC_EN_Pin GPIO_PIN_0
#define MC_EN_GPIO_Port GPIOF
#define MC_DIR_Pin GPIO_PIN_1
#define MC_DIR_GPIO_Port GPIOF
#define ML_EN_Pin GPIO_PIN_2
#define ML_EN_GPIO_Port GPIOF
#define ML_DIR_Pin GPIO_PIN_3
#define ML_DIR_GPIO_Port GPIOF
#define MR_EN_Pin GPIO_PIN_4
#define MR_EN_GPIO_Port GPIOF
#define MR_DIR_Pin GPIO_PIN_5
#define MR_DIR_GPIO_Port GPIOF
#define TIM10_CH1_MC_PWM_Pin GPIO_PIN_6
#define TIM10_CH1_MC_PWM_GPIO_Port GPIOF
#define TOUCH_0_Pin GPIO_PIN_7
#define TOUCH_0_GPIO_Port GPIOE
#define TOUCH_1_Pin GPIO_PIN_8
#define TOUCH_1_GPIO_Port GPIOE
#define TOUCH_2_Pin GPIO_PIN_9
#define TOUCH_2_GPIO_Port GPIOE
#define TOUCH_3_Pin GPIO_PIN_10
#define TOUCH_3_GPIO_Port GPIOE
#define TOUCH_4_Pin GPIO_PIN_12
#define TOUCH_4_GPIO_Port GPIOE
#define TOUCH_5_Pin GPIO_PIN_13
#define TOUCH_5_GPIO_Port GPIOE
#define TOUCH_6_Pin GPIO_PIN_14
#define TOUCH_6_GPIO_Port GPIOE
#define TOUCH_7_Pin GPIO_PIN_15
#define TOUCH_7_GPIO_Port GPIOE
#define TIM12_CH1_MR_PWM_Pin GPIO_PIN_14
#define TIM12_CH1_MR_PWM_GPIO_Port GPIOB
#define TIM12_CH2_ML_PWM_Pin GPIO_PIN_15
#define TIM12_CH2_ML_PWM_GPIO_Port GPIOB
#define WING_R_RED_Pin GPIO_PIN_10
#define WING_R_RED_GPIO_Port GPIOD
#define WING_R_GREEN_Pin GPIO_PIN_14
#define WING_R_GREEN_GPIO_Port GPIOD
#define WING_R_BLUE_Pin GPIO_PIN_15
#define WING_R_BLUE_GPIO_Port GPIOD
#define GYRO_I2C3_SDA_Pin GPIO_PIN_9
#define GYRO_I2C3_SDA_GPIO_Port GPIOC
#define GYRO_I2C3_SCL_Pin GPIO_PIN_8
#define GYRO_I2C3_SCL_GPIO_Port GPIOA
#define WING_L_RED_Pin GPIO_PIN_13
#define WING_L_RED_GPIO_Port GPIOH
#define WING_L_GREEN_Pin GPIO_PIN_14
#define WING_L_GREEN_GPIO_Port GPIOH
#define WING_L_BLUE_Pin GPIO_PIN_15
#define WING_L_BLUE_GPIO_Port GPIOH
#define DIS_SCL_Pin GPIO_PIN_0
#define DIS_SCL_GPIO_Port GPIOI
#define DIS_SDA_Pin GPIO_PIN_1
#define DIS_SDA_GPIO_Port GPIOI
#define DIS_CS1_Pin GPIO_PIN_2
#define DIS_CS1_GPIO_Port GPIOI
#define DIS_CS2_Pin GPIO_PIN_3
#define DIS_CS2_GPIO_Port GPIOI
#define DIS_CS3_Pin GPIO_PIN_15
#define DIS_CS3_GPIO_Port GPIOA
#define DIS2_CS_6_Pin GPIO_PIN_12
#define DIS2_CS_6_GPIO_Port GPIOC
#define DIS2_CS_5_Pin GPIO_PIN_0
#define DIS2_CS_5_GPIO_Port GPIOD
#define DIS2_CS_4_Pin GPIO_PIN_1
#define DIS2_CS_4_GPIO_Port GPIOD
#define DIS2_CS_3_Pin GPIO_PIN_2
#define DIS2_CS_3_GPIO_Port GPIOD
#define DIS2_CS_2_Pin GPIO_PIN_3
#define DIS2_CS_2_GPIO_Port GPIOD
#define DIS2_CS_1_Pin GPIO_PIN_4
#define DIS2_CS_1_GPIO_Port GPIOD
#define DIS2_CS_0_Pin GPIO_PIN_5
#define DIS2_CS_0_GPIO_Port GPIOD
#define DIS2_SDA_Pin GPIO_PIN_6
#define DIS2_SDA_GPIO_Port GPIOD
#define DIS2_SCL_Pin GPIO_PIN_7
#define DIS2_SCL_GPIO_Port GPIOD
#define UART_F1_TX_Pin GPIO_PIN_9
#define UART_F1_TX_GPIO_Port GPIOG
#define BOTTOM_RED_Pin GPIO_PIN_12
#define BOTTOM_RED_GPIO_Port GPIOG
#define BOTTOM_GREEN_Pin GPIO_PIN_13
#define BOTTOM_GREEN_GPIO_Port GPIOG
#define UART_F1_TXG14_Pin GPIO_PIN_14
#define UART_F1_TXG14_GPIO_Port GPIOG
#define BOTTOM_BLUE_Pin GPIO_PIN_15
#define BOTTOM_BLUE_GPIO_Port GPIOG
#define IR_IN2_Pin GPIO_PIN_3
#define IR_IN2_GPIO_Port GPIOB
#define IR_IN_Pin GPIO_PIN_5
#define IR_IN_GPIO_Port GPIOB
#define BMS_I2C1_SCL_Pin GPIO_PIN_6
#define BMS_I2C1_SCL_GPIO_Port GPIOB
#define BMS_I2C1_SDA_Pin GPIO_PIN_7
#define BMS_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
