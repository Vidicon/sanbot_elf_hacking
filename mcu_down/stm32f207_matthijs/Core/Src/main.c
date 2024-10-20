/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "protocol_0x55.h"
#include "RGBLeds.h"
#include "RobotGlobals.h"
#include "Arms.h"
#include "Base.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
int Time20Hz = 0;
int Time16Hz = 0;
int Selftest = False;
int temp = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void System_Initialize()
{
	HAL_TIM_Base_Start_IT(&htim14);

	RGBLeds_Init();

	Encoders_Init(&huart6);

	LeftArm_Init(&htim9);
	RightArm_Init(&htim9);

	Base_Init(&htim9, &htim11, &htim12);
}

void System_SelfTest(enum ENUM_Booleans Enabled)
{
	Selftest = Enabled;

	LeftArm_Home();
	RightArm_Home();
}

void UpdateSelfTest()
{
//	if (Selftest)
//	{
//	  if (Time20Hz == 1 * UPDATE_20HZ) { LeftArm_NewSetpoint(300);}
//	  if (Time20Hz == 4 * UPDATE_20HZ) { LeftArm_NewSetpoint(0);  }
//	  if (Time20Hz == 8 * UPDATE_20HZ) { LeftArm_NewSetpoint(100);}
//	  if (Time20Hz == 10 * UPDATE_20HZ) { LeftArm_NewSetpoint(0);  }
//
//	  if (Time20Hz == 1 * UPDATE_20HZ) { RightArm_NewSetpoint(-100); }
//	  if (Time20Hz == 3 * UPDATE_20HZ) { RightArm_NewSetpoint(0); }
//	  if (Time20Hz == 7 * UPDATE_20HZ) { RightArm_NewSetpoint(-300); }
//	  if (Time20Hz == 10 * UPDATE_20HZ) { RightArm_NewSetpoint(0); }
//
//	  if (Time20Hz == 12 * UPDATE_20HZ)
//	  {
//		  RGBLeds_SetColorOff(LeftArm);
//		  RGBLeds_SetColorOff(RightArm);
//	  }
//	}

//	int Vel = 30;
//	int Acc = 1;
//
//	if ((Time20Hz % 300) == 0) 		{ Base_VelocitySetpoint(0, 0, Vel);}
//	if ((Time20Hz % 300) == 60) 	{ Base_VelocitySetpoint(0, 0, 0);}
//	if ((Time20Hz % 300) == 120) 	{ Base_VelocitySetpoint(0, 0, -Vel);}
//	if ((Time20Hz % 300) == 180) 	{ Base_VelocitySetpoint(0, 0, 0);}
}

void Check_USB_Communication()
{
	if (Protocol_0x55_CheckFifo() > 0)
	{
		int command = Protocol_0x55_GetCommand();

		if (command == CMD_VERSION) 	{ SendVersion();}
		if (command == CMD_LA_COLOR) 	{ RGBLeds_SetAllColors(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_RA_COLOR) 	{ RGBLeds_SetAllColors(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_BASE_COLOR) 	{ RGBLeds_SetAllColors(Base, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}

		if (command == CMD_BA_COLOR)
		{
			RGBLeds_SetAllColors(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
			RGBLeds_SetAllColors(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_LA_MOVE) 	{ Arm_PositionSetpoint(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_RA_MOVE) 	{ Arm_PositionSetpoint(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}

		if (command == CMD_BASE_MOVE)
		{
			Base_VelocitySetpoint(Protocol_0x55_GetData(3), Protocol_0x55_GetData(4), Protocol_0x55_GetData(5));
		}


		Protocol_0x55_MarkProcessed();
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

  System_Initialize();
  System_SelfTest(False);

  Protocol_0x55_Init();

  GenericBase_HAL_Brake(False, LeftBaseMotor);
  GenericBase_HAL_Brake(False, CenterBaseMotor);
  GenericBase_HAL_Brake(False, RightBaseMotor);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Update_20Hz)
	  {
		  Update_20Hz = 0;
		  Time20Hz += 1;

		  UpdateSelfTest();

		  Base_Update20Hz(Encoders_GetPointer_New());
		  Arms_Update20Hz(Encoders_GetPointer_New());

		  TracingUpdate();
	  }

	  if (Update_10Hz)
	  {
		  Update_10Hz = 0;
		  RGBLeds_Update10Hz();

//		  temp = HAL_GPIO_ReadPin(PG10_GPIO_Port, PG10_Pin);
//		  temp = HAL_GPIO_ReadPin(PG11_GPIO_Port, PG11_Pin);
//
//		  temp = HAL_GPIO_ReadPin(PD6_GPIO_Port, PD6_Pin);
//		  temp = HAL_GPIO_ReadPin(PD7_GPIO_Port, PD7_Pin);
	  }

	  if (Update_5Hz)
	  {
		  Update_5Hz = 0;
	  }

	  if (Update_2Hz)
	  {
		  Update_2Hz = 0;

		  SendEncoders(Encoders_GetPointer_New());
	  }

	  Check_USB_Communication();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 15;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 15;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 100;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 15;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1599;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LeftArmBrake_Pin|RightArmUp_Pin|RightArmBrake_Pin|LeftArmUp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CenterBrake_Pin|CenterDir_Pin|LeftBrake_Pin|LeftDir_Pin
                          |RightBrake_Pin|RightDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RightArmRed_Pin|RightArmGreen_Pin|RightArmBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LeftArmRed_Pin|LeftArmGreen_Pin|LeftArmBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BaseRed_Pin|BaseGreen_Pin|BaseBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LeftArmBrake_Pin RightArmUp_Pin RightArmBrake_Pin LeftArmUp_Pin */
  GPIO_InitStruct.Pin = LeftArmBrake_Pin|RightArmUp_Pin|RightArmBrake_Pin|LeftArmUp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CenterBrake_Pin CenterDir_Pin LeftBrake_Pin LeftDir_Pin
                           RightBrake_Pin RightDir_Pin */
  GPIO_InitStruct.Pin = CenterBrake_Pin|CenterDir_Pin|LeftBrake_Pin|LeftDir_Pin
                          |RightBrake_Pin|RightDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13_Pin PF14_Pin PF15_Pin */
  GPIO_InitStruct.Pin = PF13_Pin|PF14_Pin|PF15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RightLimitBack_Pin RightLimitUp_Pin LeftLimitUp_Pin LeftLimitBack_Pin */
  GPIO_InitStruct.Pin = RightLimitBack_Pin|RightLimitUp_Pin|LeftLimitUp_Pin|LeftLimitBack_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7_Pin PE8_Pin PE9_Pin */
  GPIO_InitStruct.Pin = PE7_Pin|PE8_Pin|PE9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RightArmRed_Pin RightArmGreen_Pin RightArmBlue_Pin */
  GPIO_InitStruct.Pin = RightArmRed_Pin|RightArmGreen_Pin|RightArmBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LeftArmRed_Pin LeftArmGreen_Pin LeftArmBlue_Pin */
  GPIO_InitStruct.Pin = LeftArmRed_Pin|LeftArmGreen_Pin|LeftArmBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PD6_Pin PD7_Pin */
  GPIO_InitStruct.Pin = PD6_Pin|PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BaseRed_Pin BaseGreen_Pin BaseBlue_Pin */
  GPIO_InitStruct.Pin = BaseRed_Pin|BaseGreen_Pin|BaseBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
