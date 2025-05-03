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
#include "Encoders.h"
#include "HeadMotors.h"
#include "SSD1305_eyes.h"
#include "default_eyes.h"
#include "fire_eyes.h"
#include "TouchSensors.h"
#include "Eyes.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HeadLed(int LedOn);
//void Update_Eyes(int BlinkEye);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
int SelfTestTimer = 0;

int Update_25Hz;
int Update_20Hz;
int Update_16Hz;
int Update_10Hz;
int Update_5Hz;
int Update_2Hz;
int Update_1Hz;

int Selftest = False;

int HeadButtonOld;
int HeadButton;

int tmp1;
int tmp2;

int head_pan_max;
int Counter_2Hz = 0;

int SlowCounter = 0;

int System_Ready = False;

int Demo_LedModeOld = 0;
int DemoEyesMode = 0;

OLED_HandleTypeDef left_eye;
OLED_HandleTypeDef right_eye;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void System_Initialize_Start()
{
	// Main timer
	HAL_TIM_Base_Start_IT(&htim7);

	RGBLeds_Init();

	Encoders_Init(&htim1, &htim3);

	Head_Pan_Init(&htim8);
//	Head_Pan_Home();

	Head_Tilt_Init(&htim8);
//	Head_Tilt_Home();

	System_Ready = True;

	HeadLed(False);

	TouchSensors_Init();

	left_eye.hspi = &hspi2;
	left_eye.cs = toGPIO(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin);
	left_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
	left_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);

	right_eye.hspi = &hspi3;
	right_eye.cs = toGPIO(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin);
	right_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
	right_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);

	SSD1305_init(&left_eye, &right_eye);

	SSD1305_writeDisplay(&left_eye, &default_left_eye_closed);
	SSD1305_writeDisplay(&right_eye, &default_right_eye_closed);

	Eyes_Init(&left_eye, &right_eye);
}

void System_Initialze_Update()
{
//	if (HeadPan_State.HomeState != Homed)
//	{
//		return;
//	}
//
//	if (HeadTilt_State.HomeState != Homed)
//	{
//		return;
//	}

	System_Ready = True;
}

void Check_USB_Communication()
{
	if (Protocol_0x55_CheckFifo() > 0)
	{
		int command = Protocol_0x55_GetCommand();

		if (command == CMD_VERSION_HEAD)
		{
			SendVersion();
		}

		if (command == CMD_HEAD_LEFT_COLOR)
		{
			RGBLeds_SetAllColors(LeftHead, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_HEAD_RIGHT_COLOR)
		{
			RGBLeds_SetAllColors(RightHead, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_HEAD_TILT_HOME)
		{
			Head_Tilt_Home();
		}

		if (command == CMD_HEAD_PAN_HOME)
		{
			Head_Pan_Home();
		}

		if (command == CMD_HEAD_PAN_MOVE)
		{
			Generic_Head_Position_Setpoint(HeadPan, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_HEAD_TILT_MOVE)
		{
			Generic_Head_Position_Setpoint(HeadTilt, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_HEAD_EYES)
		{
			Eyes_Select(Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		Protocol_0x55_MarkProcessed();
	}
}

void HeadLed(int LedOn)
{
	if (LedOn)
	{
		HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, GPIO_PIN_SET);
	}
}


int ReadHeadButton()
{
	return (HAL_GPIO_ReadPin(HeadTopButton_GPIO_Port, HeadTopButton_Pin) == GPIO_PIN_RESET);
}

//void RunDemoProgram1()
//{
//	if (System_Ready == True)
//	{
//		HeadButtonOld = HeadButton;
//		HeadButton = ReadHeadButton();
//
//		if ((HeadButtonOld == 0) && (HeadButton == 1))
//		{
//			RGBLeds_BlinkColor(LeftHead, Red, LED_Blink_Slow);
//			RGBLeds_BlinkColor(RightHead, Blue, LED_Blink_Fast);
//
//			DemoEyesMode = 1;
//			HeadLed(True);
//		}
//
//		if ((HeadButtonOld == 1) && (HeadButton == 0))
//		{
//			RGBLeds_SetAllColors(LeftHead, Blue, LED_Blink_VeryFast);
//			RGBLeds_SetAllColors(RightHead, Blue, LED_Blink_VeryFast);
//
//			DemoEyesMode = 0;
//			HeadLed(False);
//
//			Head_Pan_Home();
//			Head_Tilt_Home();
//		}
//
//		if (TouchSensor_AnyPressed())
//		{
//			// Top right of head
//			if (TouchSensorData.Sensor[4] == 1)
//			{
//				DemoEyesMode = 2;
//
//				Generic_Head_Position_Setpoint(HeadPan, 2, 128);
//				Generic_Head_Position_Setpoint(HeadTilt, 1, 128);
//
//				RGBLeds_SetAllColors(LeftHead, Red, LED_Blink_Fast);
//				RGBLeds_SetAllColors(RightHead, Red, LED_Blink_Fast);
//
//			}
//
//			// Top left of head
//			if (TouchSensorData.Sensor[0] == 1)
//			{
//				DemoEyesMode = 0;
//
//				Generic_Head_Position_Setpoint(HeadPan, 1, 0);
//				Generic_Head_Position_Setpoint(HeadTilt, 1, 128);
//
//				RGBLeds_SetAllColors(LeftHead, White, LED_Blink_Slow);
//				RGBLeds_SetAllColors(RightHead, White, LED_Blink_Slow);
//			}
//		}
//
//		// Update eyes on change
//		if (DemoEyesMode != Demo_LedModeOld)
//		{
//			Update_Eyes(False);
//		}
//
//		Demo_LedModeOld = DemoEyesMode;
//	}
//}

//void Update_Eyes(int BlinkEye)
//{
//	if (DemoEyesMode == 0)
//	{
//		if (BlinkEye == True)
//		{
//			// Change eye to normal eyes
//			SSD1305_writeDisplay(&left_eye, &default_left_eye_closed);
//			SSD1305_writeDisplay(&right_eye, &default_right_eye_closed);
//		}
//		else
//		{
//			// Change eye to normal eyes
//			SSD1305_writeDisplay(&left_eye, &default_left_eye_open);
//			SSD1305_writeDisplay(&right_eye, &default_right_eye_open);
//		}
//	}
//
//	if (DemoEyesMode == 1)
//	{
//		if (BlinkEye == True)
//		{
//			SSD1305_writeDisplay(&left_eye, &nobleo_logo);
//			SSD1305_writeDisplay(&right_eye, &sara_logo);
//
//			HeadLed(False);
//		}
//		else
//		{
//			SSD1305_writeDisplay(&left_eye, &sara_logo);
//			SSD1305_writeDisplay(&right_eye, &nobleo_logo);
//
//			HeadLed(False);
//		}
//	}
//
//	if (DemoEyesMode == 2)
//	{
//		if (BlinkEye == True)
//		{
//			SSD1305_writeDisplay(&left_eye, &fire_left_eye_low);
//			SSD1305_writeDisplay(&right_eye, &fire_right_eye_low);
//		}
//		else
//		{
//			SSD1305_writeDisplay(&left_eye, &fire_left_eye_high);
//			SSD1305_writeDisplay(&right_eye, &fire_right_eye_high);
//		}
//	}
//}

void setPanMotor(int8_t setSpeed)
{
	HAL_GPIO_WritePin(PanDirection_GPIO_Port, PanDirection_Pin, setSpeed<0); // PAN
	TIM8->CCR4 = 100 - abs(setSpeed); // PAN
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	TIM2->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	System_Initialize_Start();
	System_Initialze_Update();

	Protocol_0x55_Init();

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
		  System_Initialze_Update();

		  Head_Update20Hz(Encoders_GetPointer());
	  }

	  if (Update_10Hz)
	  {
		  Update_10Hz = 0;

		  RGBLeds_Update10Hz();
		  Encoders_Update();
		  TouchSensors_Update();

#ifdef DEMO1
		  RunDemoProgram1();
#endif
	  }

	  if (Update_5Hz)
	  {
		  Update_5Hz = 0;
	  }

	  if (Update_2Hz)
	  {
		  Update_2Hz = 0;

		  if (Counter_2Hz == 0)
		  {
			  Update_Eyes(False);
		  }

		  if (Counter_2Hz == 5)
		  {
			  Update_Eyes(True);
		  }

		  Counter_2Hz = (Counter_2Hz + 1) % 6;
	  }

	if (Update_1Hz)
	{
		Update_1Hz = 0;

		SlowCounter += 1;

#ifdef DEMO2
		if (SlowCounter % 10 == 0)
		{
//			Generic_Head_Position_Setpoint(HeadPan, 2, 128);
//			Generic_Head_Position_Setpoint(HeadTilt, 1, 128);

			RGBLeds_SetAllColors(LeftHead, Red, LED_On);
			RGBLeds_SetAllColors(RightHead, Red, LED_On);

			DemoEyesMode = 0;
		}

		if (SlowCounter % 20 == 0)
		{
//			Generic_Head_Position_Setpoint(HeadPan, 1, 64);
//			Generic_Head_Position_Setpoint(HeadTilt, 0, 128);

			RGBLeds_SetAllColors(LeftHead, White, LED_On);
			RGBLeds_SetAllColors(RightHead, White, LED_On);

			DemoEyesMode = 1;
		}

		if (SlowCounter % 30 == 0)
		{
//			Generic_Head_Position_Setpoint(HeadPan, 0, 100);
//			Generic_Head_Position_Setpoint(HeadTilt, 2, 0);

			RGBLeds_SetAllColors(LeftHead, Green, LED_On);
			RGBLeds_SetAllColors(RightHead, Green, LED_On);

			DemoEyesMode = 2;
		}
#endif
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2399;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HeadLedEnable_GPIO_Port, HeadLedEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_ENABLE_LOW_GPIO_Port, USB_ENABLE_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LeftHeadRed_Pin|LeftHeadGreen_Pin|LeftHeadBlue_Pin|TiltEnable_Pin
                          |PanEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TiltDirection_Pin|PanDirection_Pin|RightHeadRed_Pin|RightHeadGreen_Pin
                          |RightHeadBlue_Pin|OLED_RESET_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Touch2_Pin Touch3_Pin Touch4_Pin Touch5_Pin
                           Touch6_Pin Touch0_Pin Touch1_Pin */
  GPIO_InitStruct.Pin = Touch2_Pin|Touch3_Pin|Touch4_Pin|Touch5_Pin
                          |Touch6_Pin|Touch0_Pin|Touch1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PanPosSensor_Pin PanNegSensor_Pin TiltPosSensor_Pin TiltNegSensor_Pin */
  GPIO_InitStruct.Pin = PanPosSensor_Pin|PanNegSensor_Pin|TiltPosSensor_Pin|TiltNegSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

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

  /*Configure GPIO pins : TiltEnable_Pin PanEnable_Pin */
  GPIO_InitStruct.Pin = TiltEnable_Pin|PanEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_L_CS_Pin */
  GPIO_InitStruct.Pin = OLED_L_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_L_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TiltDirection_Pin PanDirection_Pin OLED_RESET_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = TiltDirection_Pin|PanDirection_Pin|OLED_RESET_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RightHeadRed_Pin RightHeadGreen_Pin RightHeadBlue_Pin */
  GPIO_InitStruct.Pin = RightHeadRed_Pin|RightHeadGreen_Pin|RightHeadBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_R_CS_Pin */
  GPIO_InitStruct.Pin = OLED_R_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_R_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Touch7_Pin */
  GPIO_InitStruct.Pin = Touch7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Touch7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
