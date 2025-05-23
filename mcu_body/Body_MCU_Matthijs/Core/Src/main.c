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
#include "usbd_cdc_if.h"
#include "protocol_0x55.h"
#include "RGBLeds.h"
#include "RobotGlobals.h"
#include "Arms.h"
#include "Base.h"
#include "MotionSensors.h"
#include "DistanceSensors.h"
#include "Compass.h"
#include "Battery.h"
#include "stm32f2xx.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
int Time20Hz = 0;
int Time16Hz = 0;
int Selftest = False;
int TempCS = 0;
int Distance = 0;
char TextBuffer[100];

int Update_25Hz;
int Update_20Hz;
int Update_16Hz;
int Update_10Hz;
int Update_5Hz;
int Update_2Hz;
int Update_1Hz;

int MotionFrontOld = 0;
int MotionFront = 0;
int DemoState[2];

int System_Ready = False;

unsigned int  uart1RxCounter    		= 0;
unsigned char uart1RxChar				= 0;
unsigned char uart4RxChar				= 0;
int datalen = 1;

void BodyStop(void);
void BodyRelease(void);

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
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//------------------------------------------------------------------------------------
// Uart interrupt callback
// Start 	= 0x55
// Type  	= 0..127 (Command), high bit = response
// Length	= 0..255
// Data
// CRC Low byte
// CRC high byte
// CRC modbus = 0x104B
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		HAL_UART_RxCpltCallback_Encoders(huart);
	}

	if (huart->Instance == USART1)
	{
		Protocol_0x55_NewData((uint8_t*) &uart1RxChar, (uint32_t*) &datalen);
		HAL_UART_Receive_IT(&huart1, &uart1RxChar, 1);	// Enable next character reception
	}

	// To/From Head
	if(huart->Instance == UART4)
	{
		// STOP button pressed
		if (uart4RxChar == 0x01)
		{
			BodyStop();
		}

		// STOP button released
		if (uart4RxChar == 0x02)
		{
			BodyRelease();
		}

		HAL_UART_Receive_IT(&huart4, &uart4RxChar, 1);
	}
}
void BodyStop(void)
{
	RGBLeds_SetAllColors(LeftArm, Red, LED_On);
	RGBLeds_SetAllColors(RightArm, Red, LED_On);
	RGBLeds_SetAllColors(Base, Red, LED_On);

	Base_Abort();
	LeftArm_Abort();
	RightArm_Abort();
}

void BodyRelease(void)
{
	RGBLeds_SetAllColors(LeftArm, Red, LED_Off);
	RGBLeds_SetAllColors(RightArm, Red, LED_Off);
	RGBLeds_SetAllColors(Base, Red, LED_Off);
}

void System_Initialize()
{
	HAL_TIM_Base_Start_IT(&htim14);

	RGBLeds_Init();

	Protocol_0x55_Init(&huart1);

	HAL_UART_Receive_IT(&huart1, &uart1RxChar, 1);
	HAL_UART_Receive_IT(&huart4, &uart4RxChar, 1);

	Encoders_Init(&huart6);

	LeftArm_Init(&htim9);
	RightArm_Init(&htim9);

	LeftArm_Home();
	RightArm_Home();

	Base_Init(&htim9, &htim11, &htim12);

	MotionSensors_Init();
	DistanceSensors_Init();

	Compass_Init(&hi2c3);

	Battery_Init(&hi2c1);
}

void System_Initialze_Update()
{
	if (LeftArm_State.HomeState != Homed) {return;}
	if (RightArm_State.HomeState != Homed) {return;}

	System_Ready = True;
}

void Check_USB_Communication()
{
	if (Protocol_0x55_CheckFifo() > 0)
	{
		int command = Protocol_0x55_GetCommand();

		if (command == CMD_VERSION) 	{ SendVersion();}
		if (command == CMD_LA_COLOR)	{ RGBLeds_SetAllColors(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_RA_COLOR) 	{ RGBLeds_SetAllColors(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_BASE_COLOR) 	{ RGBLeds_SetAllColors(Base, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}

		if (command == CMD_LARA_COLOR)
		{
			RGBLeds_SetAllColors(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
			RGBLeds_SetAllColors(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}


		if (command == CMD_BA_COLOR)
		{
			RGBLeds_SetAllColors(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
			RGBLeds_SetAllColors(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_LA_MOVE) 	{ Generic_Arm_PositionSetpoint(LeftArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}
		if (command == CMD_RA_MOVE) 	{ Generic_Arm_PositionSetpoint(RightArm, Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));}

		if (command == CMD_BASE_MOVE)
		{
			Base_MotionStartWatchdog(Protocol_0x55_GetData(6));
			Base_VelocitySetpoint(Protocol_0x55_GetData(3), Protocol_0x55_GetData(4), Protocol_0x55_GetData(5));
		}

		if (command == CMD_BASE_BRAKE)
		{
			Base_Brake(Protocol_0x55_GetData(3));
		}

		if (command == CMD_COMP_MOVE)
		{
			Base_MotionStartWatchdog(Protocol_0x55_GetData(5));
			Base_NewCompassRotation(Protocol_0x55_GetData(3), Protocol_0x55_GetData(4));
		}

		if (command == CMD_LA_HOME)
		{
			LeftArm_Home();
		}

		if (command == CMD_RA_HOME)
		{
			RightArm_Home();
		}

		if (command == CMD_BODY_STOP)
		{
			BodyStop();
		}

		Protocol_0x55_MarkProcessed();
	}
}

void TracingUpdate()
{
	memset(TextBuffer, 0x00, 100);

	sprintf(TextBuffer, "%ld\n",(long)(Distance));
	CDC_Transmit_FS((uint8_t*)TextBuffer, strlen(TextBuffer));
}

int ReadMotionFront()
{
	struct MotionSensors_Data_Type *Tmp;
	Tmp = MotionSensors_GetPointer();

	return (Tmp->CurrentValue[0] == 1);
}

int ReadDistancesFrontLeft()
{
	struct Distance_Sensor_Type *Tmp;
	Tmp = DistanceSensors_GetPointer();

	int result = 0;

	for (int i = 2; i < 4; i++)
	{
		if (Tmp->Distance[i] <= 50000) { result = 1;}
	}

	return result;
}

int ReadDistancesFrontRight()
{
	struct Distance_Sensor_Type *Tmp;
	Tmp = DistanceSensors_GetPointer();

	int result = 0;

	for (int i = 4; i < 6; i++)
	{
		if (Tmp->Distance[i] <= 50000) { result = 1;}
	}

	return result;
}

int ReadDistancesLeftMid()
{
	struct Distance_Sensor_Type *Tmp;
	Tmp = DistanceSensors_GetPointer();

	return (Tmp->Distance[9] <= 60000);
}

int ReadDistancesRightMid()
{
	struct Distance_Sensor_Type *Tmp;
	Tmp = DistanceSensors_GetPointer();

	return (Tmp->Distance[10] <= 60000);
}

void RunDemoProgram()
{
	MotionFrontOld = MotionFront;
	MotionFront = ReadMotionFront();

	if (System_Ready == True)
	{
		//-----------------------------------------------------------------------------------
		// Left arm
		//-----------------------------------------------------------------------------------
		if (DemoState[0] == 0)
		{
			if (ReadDistancesLeftMid() || ReadDistancesFrontLeft())
			{
				DemoState[0] = 1;

				RGBLeds_SetAllColors(LeftArm, Blue, LED_Blink_VeryFast);
				Generic_Arm_PositionSetpoint(LeftArm, 1, 150);

				RGBLeds_SetAllColors(Base, Red, LED_Blink_VeryFast);
			}
		}
		else if (DemoState[0] == 1)
		{
			if ((LeftArm_State.MotionState == Motion_Idle) && (ReadDistancesFrontLeft() == 0))
			{
				DemoState[0] = 2;
				RGBLeds_SetAllColors(LeftArm, Green, LED_On);
				RGBLeds_SetAllColors(Base, Blue, LED_On);
				Generic_Arm_PositionSetpoint(LeftArm, 1, 0);
			}
		}
		else if (DemoState[0] == 2)
		{
			if (LeftArm_State.MotionState == Motion_Idle)
			{
				DemoState[0] = 0;
			}
		}

		//-----------------------------------------------------------------------------------
		// Right arm
		//-----------------------------------------------------------------------------------
		if (DemoState[1] == 0)
		{
			if (ReadDistancesRightMid() || ReadDistancesFrontRight())
			{
				DemoState[1] = 1;

				RGBLeds_SetAllColors(RightArm, Blue, LED_Blink_VeryFast);
				RGBLeds_SetAllColors(Base, Blue, LED_Blink_VeryFast);
				Generic_Arm_PositionSetpoint(RightArm, 1, 150);
			}
		}
		else if (DemoState[1] == 1)
		{
			if ((RightArm_State.MotionState == Motion_Idle) && (ReadDistancesFrontRight() == 0))
			{
				DemoState[1] = 2;
				RGBLeds_SetAllColors(RightArm, Green, LED_On);
				RGBLeds_SetAllColors(Base, Blue, LED_On);
				Generic_Arm_PositionSetpoint(RightArm, 1, 0);
			}
		}
		else if (DemoState[1] == 2)
		{
			if (RightArm_State.MotionState == Motion_Idle)
			{
				DemoState[1] = 0;
			}
		}
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
  MX_TIM14_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  System_Initialize();

  GenericBase_HAL_Brake(False, LeftBaseMotor);
  GenericBase_HAL_Brake(False, CenterBaseMotor);
  GenericBase_HAL_Brake(False, RightBaseMotor);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Update_25Hz)
	  {
		  Update_25Hz = 0;
		  DistanceSensors_Update();
	  }

	  if (Update_20Hz)
	  {
		  Update_20Hz = 0;
		  Time20Hz += 1;

		  System_Initialze_Update();

		  Base_Update20Hz(Encoders_GetPointer());
		  Arms_Update20Hz(Encoders_GetPointer());
	  }

	  if (Update_10Hz)
	  {
		  Update_10Hz = 0;

		  RGBLeds_Update10Hz();
		  MotionSensors_Update10Hz();

		  Compass_Update();

		  Base_MotionUpdateWatchdog();
		  Base_MotionControl(Compass_GetPointer());

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

		  SendEncoders(Encoders_GetPointer());
		  HAL_Delay(1);
		  SendCompass(Compass_GetPointer());
		  HAL_Delay(1);
	  }

	  if (Update_1Hz)
	  {
		  Update_1Hz = 0;

		  Battery_Update();

		  Compass_Init(&hi2c3);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LeftArmBrake_Pin|RightArmUp_Pin|RightArmBrake_Pin|LeftArmUp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CenterBrake_Pin|CenterDir_Pin|LeftBrake_Pin|LeftDir_Pin
                          |RightBrake_Pin|RightDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCL_Distance_J26_Pin|EN1_Distance_J26_Pin|EN2_Distance_J26_Pin|SCL_Distance_J22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN3_Distance_J26_Pin|EN4_Distance_J26_Pin|EN1_Distance_J28_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN2_Distance_J22_Pin|EN1_Distance_J22_Pin|SCL_Distance_J18_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RightArmRed_Pin|RightArmGreen_Pin|RightArmBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LeftArmRed_Pin|LeftArmGreen_Pin|LeftArmBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, SCL_Distance_MID_Pin|EN1_Distance_J21_Pin|EN1_Distance_J24_Pin|EN1_Distance_J18_Pin
                          |EN2_Distance_J18_Pin|EN3_Distance_J18_Pin|EN4_Distance_J18_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SCL_Distance_J26_Pin SCL_Distance_J22_Pin */
  GPIO_InitStruct.Pin = SCL_Distance_J26_Pin|SCL_Distance_J22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA_Distance_J26_Pin SDA_Distance_J22_Pin */
  GPIO_InitStruct.Pin = SDA_Distance_J26_Pin|SDA_Distance_J22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Distance_J26_Pin EN2_Distance_J26_Pin */
  GPIO_InitStruct.Pin = EN1_Distance_J26_Pin|EN2_Distance_J26_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN3_Distance_J26_Pin EN4_Distance_J26_Pin EN1_Distance_J28_Pin */
  GPIO_InitStruct.Pin = EN3_Distance_J26_Pin|EN4_Distance_J26_Pin|EN1_Distance_J28_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN2_Distance_J22_Pin EN1_Distance_J22_Pin */
  GPIO_InitStruct.Pin = EN2_Distance_J22_Pin|EN1_Distance_J22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13_Pin PF14_Pin PF15_Pin */
  GPIO_InitStruct.Pin = PF13_Pin|PF14_Pin|PF15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RightLimitBack_Pin RightLimitUp_Pin MotionBack_Pin MotionFront_Pin
                           LeftLimitUp_Pin LeftLimitBack_Pin */
  GPIO_InitStruct.Pin = RightLimitBack_Pin|RightLimitUp_Pin|MotionBack_Pin|MotionFront_Pin
                          |LeftLimitUp_Pin|LeftLimitBack_Pin;
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

  /*Configure GPIO pin : SCL_Distance_MID_Pin */
  GPIO_InitStruct.Pin = SCL_Distance_MID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SCL_Distance_MID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDA_Distance_MID_Pin */
  GPIO_InitStruct.Pin = SDA_Distance_MID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDA_Distance_MID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Distance_J21_Pin EN1_Distance_J24_Pin EN1_Distance_J18_Pin EN2_Distance_J18_Pin
                           EN3_Distance_J18_Pin EN4_Distance_J18_Pin */
  GPIO_InitStruct.Pin = EN1_Distance_J21_Pin|EN1_Distance_J24_Pin|EN1_Distance_J18_Pin|EN2_Distance_J18_Pin
                          |EN3_Distance_J18_Pin|EN4_Distance_J18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

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

  /*Configure GPIO pin : SCL_Distance_J18_Pin */
  GPIO_InitStruct.Pin = SCL_Distance_J18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SCL_Distance_J18_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDA_Distance_J18_Pin */
  GPIO_InitStruct.Pin = SDA_Distance_J18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDA_Distance_J18_GPIO_Port, &GPIO_InitStruct);

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
