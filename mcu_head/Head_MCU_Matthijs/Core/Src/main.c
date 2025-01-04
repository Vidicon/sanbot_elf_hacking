/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol_0x55.h"
#include "RobotGlobals.h"
#include "RGBLeds_Head.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HeadLed(int Enable);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
int SelfTestTimer = 0;
int Time20Hz = 0;
int Time16Hz = 0;
int Selftest = False;

int HeadButtonOld;
int HeadButton;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void System_Initialize()
{
	// Main timer
	HAL_TIM_Base_Start_IT(&htim7);

//	RGBLeds_Init();

//	Encoders_Init(&huart6);
//
//	LeftArm_Init(&htim9);
//	RightArm_Init(&htim9);
//
//	Base_Init(&htim9, &htim11, &htim12);
//
//	MotionSensors_Init();
//	DistanceSensors_Init();
//
//	Compass_Init(&hi2c3);
//
//	Battery_Init(&hi2c1);
}

void System_SelfTest(enum ENUM_Booleans Enabled)
{
	Selftest = Enabled;

	if (Selftest)
	{
		SelfTestTimer = 0;
		RGBLeds_SelfTest(True);
		HeadLed(True);
	}

//	LeftArm_Home();
//	RightArm_Home();
}

void UpdateSelfTest()
{
	if (Selftest)
	{
		SelfTestTimer += 1;
		if (SelfTestTimer == 5 * UPDATE_20HZ)
		{
			SelfTestTimer = 0;
			Selftest = False;
			RGBLeds_SelfTest(False);
			HeadLed(False);
		}
	}
}

void Check_USB_Communication()
{
	if (Protocol_0x55_CheckFifo() > 0)
	{
		int command = Protocol_0x55_GetCommand();

		if (command == CMD_VERSION)
		{
			SendVersion();
		}

		if (command == CMD_LEFTHEAD_COLOR)
		{
			RGBLeds_SetAllColors(LeftHead, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_RIGHTHEAD_COLOR)
		{
			RGBLeds_SetAllColors(RightHead, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		Protocol_0x55_MarkProcessed();
	}
}

void HeadLed(int Enable)
{
	// Low to enable headled
	// 99 = almost off
	// 0 = full on.
	if (Enable)
	{
		HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, 0);
		TIM2->CCR2 = 70;
	}
	else
	{
		HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, 1);
		TIM2->CCR2 = 99;
	}
}

int ReadHeadButton()
{
	return (HAL_GPIO_ReadPin(HeadTopButton_GPIO_Port, HeadTopButton_Pin) == GPIO_PIN_RESET);
}

void RunDemoProgram()
{
	if (Selftest == 0)
	{
		HeadButtonOld = HeadButton;
		HeadButton = ReadHeadButton();

		if ((HeadButtonOld == 0) && (HeadButton == 1))
		{
			RGBLeds_BlinkColor(LeftHead, Red, LED_Blink_Slow);
			RGBLeds_BlinkColor(RightHead, Green, LED_Blink_Fast);
			HeadLed(True);

			// Change eye to Nobleo + Sara logo
		}

		if ((HeadButtonOld == 1) && (HeadButton == 0))
		{
			RGBLeds_SetAllColors(LeftHead, Blue, LED_On);
			RGBLeds_SetAllColors(RightHead, Blue, LED_On);
			HeadLed(False);

			// Change eye to normal eyes
		}

//		if (ReadTouch() == 1)
//		{
//			// If touched left, close left eye
			// If touched right, close right eye
//		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  TIM2->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  System_Initialize();
  System_SelfTest(True);

  Protocol_0x55_Init();

  HeadLed(True);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Update_25Hz)
	  {
		  Update_25Hz = 0;
	  }

	  if (Update_20Hz)
	  {
		  Update_20Hz = 0;
		  UpdateSelfTest();
	  }

	  if (Update_10Hz)
	  {
		  Update_10Hz = 0;

		  RGBLeds_Update10Hz();

		  #ifdef DEMO
		  RunDemoProgram();
		  #endif
	  }

	  if (Update_5Hz)
	  {
		  Update_5Hz = 0;
	  }

	  if (Update_2Hz)
	  {
		  Update_2Hz = 0;
	  }

	  if (Update_1Hz)
	  {
		  Update_1Hz = 0;

//		  SendVersion();
	  }

	  //--------------------------------------------------------
	  // Limit the check for new data frequency
	  // When run at full speed (no delay), the USB interrupt and the check will lead
	  // to lost bytes. Dont know why and dont know how to solve. But delay works fine.
	  //--------------------------------------------------------
	  Check_USB_Communication();
	  HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_ENABLE_LOW_GPIO_Port, USB_ENABLE_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LeftHeadRed_Pin|LeftHeadGreen_Pin|LeftHeadBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RightHeadRed_Pin|RightHeadGreen_Pin|RightHeadBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HeadLedEnable_Pin */
  GPIO_InitStruct.Pin = HeadLedEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HeadLedEnable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HeadTopButton_Pin */
  GPIO_InitStruct.Pin = HeadTopButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HeadTopButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_ENABLE_LOW_Pin */
  GPIO_InitStruct.Pin = USB_ENABLE_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_ENABLE_LOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LeftHeadRed_Pin LeftHeadGreen_Pin LeftHeadBlue_Pin */
  GPIO_InitStruct.Pin = LeftHeadRed_Pin|LeftHeadGreen_Pin|LeftHeadBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RightHeadRed_Pin RightHeadGreen_Pin RightHeadBlue_Pin */
  GPIO_InitStruct.Pin = RightHeadRed_Pin|RightHeadGreen_Pin|RightHeadBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
