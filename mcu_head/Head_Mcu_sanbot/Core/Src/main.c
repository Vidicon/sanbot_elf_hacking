/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include <stdio.h>
#include "SSD1305_eyes.h"

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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t displayBuffer[128*8];
uint32_t p = 0;

void getdata(uint8_t * rxBuffer, uint32_t size)
{
	HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
//	if(size > 1024+2)
//	{
//		size = 1024+2;
//	}
//	uint8_t data0 = rxBuffer[0];
//	uint8_t data1 = rxBuffer[1];
//	uint8_t data2 = rxBuffer[2];

	  if(rxBuffer[0] == 0xaa)
	  {
		  p = rxBuffer[1] * 32;
		  memcpy(&displayBuffer[p], &rxBuffer[2], size-2);
	  }
	  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setPanMotor(int8_t setSpeed)
{
	HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, setSpeed<0); // PAN
	TIM8->CCR4 = 100 - abs(setSpeed); // PAN
}

void setTilMotor(int8_t setSpeed)
{
	HAL_GPIO_WritePin(TIL_DIR_GPIO_Port, TIL_DIR_Pin, setSpeed<0); // PAN
	TIM8->CCR3 = 100 - abs(setSpeed); // PAN
}

void read_flash(uint32_t address, uint8_t * data, uint16_t size)
{
	const uint16_t msgSize = 1+3+size;
	uint8_t txdata[msgSize];
	memset(txdata, 0, msgSize);
	txdata[0] = 0x03;
	//  uint32_t address = 0x000;
	txdata[1] = (address >> 16) & 0xFF;
	txdata[2] = (address >> 8) & 0xFF;
	txdata[3] = (address >> 0) & 0xFF;

	//  uint8_t rxdata[msgSize];
	//  memset(rxdata, 0, msgSize);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, txdata, data, msgSize, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
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
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COUNTER(&htim1, 1000);
  __HAL_TIM_SET_COUNTER(&htim3, 1000);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

//  HAL_TIM_Base_Start(htim12);
  TIM8->CCR3 = 100;
  TIM8->CCR4 = 100;
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start(&htim8, );

  TIM2->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


  OLED_HandleTypeDef left_eye;
  left_eye.hspi = &hspi2;
  left_eye.cs = toGPIO(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin);
  left_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
  left_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);

  OLED_HandleTypeDef right_eye;
  right_eye.hspi = &hspi3;
  right_eye.cs = toGPIO(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin);
  right_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
  right_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);

  HAL_Delay(10);

  SSD1305_init(&left_eye, &right_eye);
//
//  for(uint8_t i = 0; i < 10; i++)
//  {
//	  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, 0);
//	  uint8_t data = i+1;
//	  HAL_SPI_Transmit(&hspi2, &data, 1, 100);
//	  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, 1);
//
//  }
//  uint32_t address = 0x001000 + (75 * 128 * 8);

  uint8_t toggle = 0;
  uint32_t f = 0x00;
  uint8_t eyelist[][2] = {{0x00,1}, {0x00,1}, {0x18,2}, {0x30,2}, {0x48,2},  {0x60,7},  {0x78,2}, {0x90,3}, {0x99,2}, {0xA8,2}, {0xC0,2}, {0xD8,2}, {0xf0,4}};
  uint8_t s = 2;
  uint8_t i = 0;
  uint8_t buffer[128*8+4];
  int x = 0;

  uint16_t head_til_max = 1;
  uint16_t head_pan_max = 1;

  uint16_t head_til_min = 1;
  uint16_t head_pan_min = 1;

  uint8_t brightness = 0;


//  HAL_GPIO_WritePin(TIL_EN_GPIO_Port, TIL_EN_Pin, 1);
//  HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, 1);
//  uint8_t calibration = 0;
//
//  while(!HAL_GPIO_ReadPin(PAN2_SENS_GPIO_Port, PAN2_SENS_Pin))
//  {
//	  setPanMotor(-30);
//	  HAL_Delay(10);
//  }
//  setPanMotor(0);
//  __HAL_TIM_SET_COUNTER(&htim3, 0);
//
//  while(!HAL_GPIO_ReadPin(PAN1_SENS_GPIO_Port, PAN1_SENS_Pin))
//  {
//	  setPanMotor(30);
//  	  HAL_Delay(10);
//  }
//  setPanMotor(0);
//  head_pan_max = __HAL_TIM_GET_COUNTER(&htim3);
//
//  float angle_pan_target = 0.5;
//
//  while(!HAL_GPIO_ReadPin(TIL2_SENS_GPIO_Port, TIL2_SENS_Pin))
//  {
//	  setTilMotor(30);
//	  HAL_Delay(10);
//  }
//  setTilMotor(0);
//  __HAL_TIM_SET_COUNTER(&htim1, 0);
//
//  while(!HAL_GPIO_ReadPin(TIL1_SENS_GPIO_Port, TIL1_SENS_Pin))
//  {
//	  setTilMotor(-30);
//  	  HAL_Delay(10);
//  }
//  setTilMotor(0);
//  head_til_max = __HAL_TIM_GET_COUNTER(&htim1);
//
//  float angle_til_target = 0.5;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t index = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//	  SSD1305_writeDisplay(&left_eye, displayBuffer);
//	  SSD1305_writeDisplay(&right_eye, displayBuffer);
//	  volatile uint16_t head_til = __HAL_TIM_GET_COUNTER(&htim1);
//	  volatile uint16_t head_pan = __HAL_TIM_GET_COUNTER(&htim3);


//	  char str[50];
//	  sprintf(str, "%d,  %d", head_til, head_pan);
////	  uint8_t message[] = "Hello, World!";
//	  HAL_Delay(10000);
//	  for(uint32_t index = 0; index < 0x1FFFFF; index+=128)
//	  {
//		  read_flash(index, buffer, 128);
//		  CDC_Transmit_FS(buffer, 128);
//		  HAL_Delay(10);
//
//	  }
//      read_flash(index, buffer, 1);
//	  CDC_Transmit_FS(str, strlen(str));

//	  if(HAL_GPIO_ReadPin(TIL1_SENS_GPIO_Port, TIL1_SENS_Pin))
//	  {
//		  head_til_min = head_til;
//	  }
//	  if(HAL_GPIO_ReadPin(TIL2_SENS_GPIO_Port, TIL2_SENS_Pin))
//	  {
//		  head_til_max = head_til;
//	  }
//
//	  if(HAL_GPIO_ReadPin(PAN1_SENS_GPIO_Port, PAN1_SENS_Pin))
//	  {
////		  __HAL_TIM_SET_COUNTER(&htim3, 0);
//	  }
//	  if(HAL_GPIO_ReadPin(PAN2_SENS_GPIO_Port, PAN2_SENS_Pin))
//	  {
//		  head_pan_max = head_pan;
//	  }
//
//	  float angle_til = 0;
//	  if(head_til_max > 0)
//		  angle_til = head_til / head_til_max;

//	  float angle_pan = 0;
//	  if(head_pan_max > 0)
//		  angle_pan = (float)head_pan / (float)head_pan_max;
//
//
//
//
//	  float errorAngle =  angle_pan_target - angle_pan;
//
//	  setPanMotor((int8_t)(errorAngle * 100));

//	  HAL_GPIO_WritePin(TIL_EN_GPIO_Port, TIL_EN_Pin, 1);
//	  HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, 1);
//
//	  if(!HAL_GPIO_ReadPin(TOUCH_90_GPIO_Port, TOUCH_90_Pin) ) //&& HAL_GPIO_ReadPin(TIL1_SENS_GPIO_Port, TIL1_SENS_Pin)
//	  {
//		  angle_pan_target +=0.1;
////		  setPanMotor(20);
//	  }
//	  else if(!HAL_GPIO_ReadPin(TOUCH_91_GPIO_Port, TOUCH_91_Pin)) // && HAL_GPIO_ReadPin(TIL2_SENS_GPIO_Port, TIL2_SENS_Pin)
//	  {
//		  angle_pan_target -=0.1;
////		  setPanMotor(-20);
//	  }
//	  else
//	  {
////		  setPanMotor(0);
//	  }
//	  if(!HAL_GPIO_ReadPin(TOUCH_90_GPIO_Port, TOUCH_90_Pin))
//	  {
//		  TIM8->CCR3 = 50; //TIL?
//
//		  HAL_GPIO_WritePin(TIL_DIR_GPIO_Port, TIL_DIR_Pin, 1);
//	  }
//	  else  if(!HAL_GPIO_ReadPin(TOUCH_91_GPIO_Port, TOUCH_91_Pin))
//	  {
//		  TIM8->CCR3 = 50; //TIL?
//		  TIM8->CCR4 = 50; // PAN
//		  HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, 0); // PAN
//		  HAL_GPIO_WritePin(TIL_DIR_GPIO_Port, TIL_DIR_Pin, 0);
//	  }
//	  else
//	  {
//		  TIM8->CCR3 = 100;
//		  TIM8->CCR4 = 100; // PAN
//	  }


//	  if(!HAL_GPIO_ReadPin(TOP_BUTTON_GPIO_Port, TOP_BUTTON_Pin))
//	  {
//
////		  HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, 1);
//
//	  }
//	  else
//	  {
//		  HAL_GPIO_WritePin(TIL_EN_GPIO_Port, TIL_EN_Pin, 0);
//		  HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, 0);
//	  }




	  if(s == 0)
	  {
		  SSD1305_writeDisplay(&left_eye, sara_logo);
		  SSD1305_writeDisplay(&right_eye, nobleo_logo);
	  }
	  else if(s == 1)
	  {
		  SSD1305_writeDisplay(&left_eye, displayBuffer);
		  SSD1305_writeDisplay(&right_eye, displayBuffer);
	  }
	  else
	  {
		  f = eyelist[s][0]+i;

		  read_flash(0x400*f, buffer, 128*8);

		  SSD1305_writeDisplay(&left_eye, &buffer[4]);


		  read_flash(0x400*(f+8), buffer, 128*8);
		  SSD1305_writeDisplay(&right_eye, &buffer[4]);
	  }


//	  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 0);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 1);
//	  HAL_Delay(100);

	  i++;
	  if(i > (eyelist[s][1]-1))
	  {
		  i = 0;
	  }

	  if(!HAL_GPIO_ReadPin(TOUCH_85_GPIO_Port, TOUCH_85_Pin))
	  {
		  if(!toggle)
		  {
			  i = 0;
			  s++;
			  if(s > 12)
			  {
				  s=0;
			  }
		  }
		  toggle = 1;
	  }
	  else  if(!HAL_GPIO_ReadPin(TOUCH_86_GPIO_Port, TOUCH_86_Pin))
	  {
		  if(!toggle)
		  {
			  i = 0;
			  s--;
			  if(s > 10)
			  {
				  s=10;
			  }
		  }
		  toggle = 1;
	  }
	  else
	  {
		  toggle = 0;
	  }



	  if(!HAL_GPIO_ReadPin(PAN1_SENS_GPIO_Port, PAN1_SENS_Pin))
	  {
		  f++;
		  HAL_GPIO_WritePin(TOP_LED_GPIO_Port, TOP_LED_Pin, 0);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(TOP_LED_GPIO_Port, TOP_LED_Pin, 1);
	  }

	  if(!HAL_GPIO_ReadPin(TOUCH_92_GPIO_Port, TOUCH_92_Pin))//top head
	  {
		  brightness++;
		  if(brightness > 70)
		  {
			  brightness = 70;
		  }
		  HAL_GPIO_WritePin(HEAD_LED_EN_GPIO_Port, HEAD_LED_EN_Pin, 0);
		  TIM2->CCR2 = 70-brightness;
	  }
	  else
	  {
		  TIM2->CCR2 = 0;
		  brightness = 0;
		  HAL_GPIO_WritePin(HEAD_LED_EN_GPIO_Port, HEAD_LED_EN_Pin, 1);
	  }


//	  TOUCH_83_GPIO_Port
	  HAL_GPIO_WritePin(LEFT_LED_R_GPIO_Port, LEFT_LED_R_Pin, HAL_GPIO_ReadPin(TOUCH_90_GPIO_Port, TOUCH_90_Pin));//left back LB
	  HAL_GPIO_WritePin(LEFT_LED_G_GPIO_Port, LEFT_LED_G_Pin, HAL_GPIO_ReadPin(TOUCH_83_GPIO_Port, TOUCH_83_Pin));//left_chin LC
	  HAL_GPIO_WritePin(LEFT_LED_B_GPIO_Port, LEFT_LED_B_Pin, HAL_GPIO_ReadPin(TOUCH_85_GPIO_Port, TOUCH_85_Pin));// Left forehead LF

	  HAL_GPIO_WritePin(RIGHT_LED_R_GPIO_Port, RIGHT_LED_R_Pin, HAL_GPIO_ReadPin(TOUCH_91_GPIO_Port, TOUCH_91_Pin));//right back RB
	  HAL_GPIO_WritePin(RIGHT_LED_G_GPIO_Port, RIGHT_LED_G_Pin, HAL_GPIO_ReadPin(TOUCH_84_GPIO_Port, TOUCH_84_Pin));//Right_chin RC
	  HAL_GPIO_WritePin(RIGHT_LED_B_GPIO_Port, RIGHT_LED_B_Pin, HAL_GPIO_ReadPin(TOUCH_86_GPIO_Port, TOUCH_86_Pin));//Right forehead RF
//	  switch(s){
//	  case 0:
//		  HAL_GPIO_WritePin(LEFT_LED_R_GPIO_Port, LEFT_LED_R_Pin, 0);
//		  HAL_GPIO_WritePin(LEFT_LED_G_GPIO_Port, LEFT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_B_GPIO_Port, LEFT_LED_B_Pin, 1);
//		  break;
//	  case 1:
//		  HAL_GPIO_WritePin(LEFT_LED_R_GPIO_Port, LEFT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_G_GPIO_Port, LEFT_LED_G_Pin, 0);
//		  HAL_GPIO_WritePin(LEFT_LED_B_GPIO_Port, LEFT_LED_B_Pin, 1);
//		  break;
//	  case 2:
//		  HAL_GPIO_WritePin(LEFT_LED_R_GPIO_Port, LEFT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_G_GPIO_Port, LEFT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_B_GPIO_Port, LEFT_LED_B_Pin, 0);
//		  break;
//	  case 3:
//		  HAL_GPIO_WritePin(RIGHT_LED_R_GPIO_Port, RIGHT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_G_GPIO_Port, RIGHT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_B_GPIO_Port, RIGHT_LED_B_Pin, 1);
//		  break;
//	  default:
//		  HAL_GPIO_WritePin(LEFT_LED_R_GPIO_Port, LEFT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_G_GPIO_Port, LEFT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(LEFT_LED_B_GPIO_Port, LEFT_LED_B_Pin, 1);
//
//		  break;
//	  }
//
//	  switch(s){
//	  case 4:
//		  HAL_GPIO_WritePin(RIGHT_LED_R_GPIO_Port, RIGHT_LED_R_Pin, 0);
//		  HAL_GPIO_WritePin(RIGHT_LED_G_GPIO_Port, RIGHT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_B_GPIO_Port, RIGHT_LED_B_Pin, 1);
//		  break;
//	  case 5:
//		  HAL_GPIO_WritePin(RIGHT_LED_R_GPIO_Port, RIGHT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_G_GPIO_Port, RIGHT_LED_G_Pin, 0);
//		  HAL_GPIO_WritePin(RIGHT_LED_B_GPIO_Port, RIGHT_LED_B_Pin, 1);
//		  break;
//	  default:
//
//		  HAL_GPIO_WritePin(RIGHT_LED_R_GPIO_Port, RIGHT_LED_R_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_G_GPIO_Port, RIGHT_LED_G_Pin, 1);
//		  HAL_GPIO_WritePin(RIGHT_LED_B_GPIO_Port, RIGHT_LED_B_Pin, 0);
//		  break;
//	  }
//
//	  HAL_GPIO_WritePin(SPI2_SCK_GPIO_Port, SPI2_SCK_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(SPI2_SCK_GPIO_Port, SPI2_SCK_Pin, 0);
////	  HAL_Delay(2000);//ok
//
//	  HAL_GPIO_WritePin(SPI3_MOSI_GPIO_Port, SPI3_MOSI_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(SPI3_MOSI_GPIO_Port, SPI3_MOSI_Pin, 0);
////	  HAL_Delay(2000);//ok
//
//	  HAL_GPIO_WritePin(SPI3_SCK_GPIO_Port, SPI3_SCK_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(SPI3_SCK_GPIO_Port, SPI3_SCK_Pin, 0);
////	  HAL_Delay(2000);//ok
//
//	  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 0);
////	  HAL_Delay(2000);//ok
//
//	  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, 0);
////	  HAL_Delay(2000);//oke
//
//	  HAL_GPIO_WritePin(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin, 0);
////	  HAL_Delay(2000);//oke
//
//	  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, 1);
////	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, 0);
//	  HAL_Delay(2000);//oke
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  htim8.Init.Prescaler = 15;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin|OneWIRE_DS18820_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TOP_LED_Pin|PAD_POWER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEAD_LED_EN_GPIO_Port, HEAD_LED_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CAM_3D_POW_EN_Pin|USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEFT_LED_R_Pin|LEFT_LED_G_Pin|LEFT_LED_B_Pin|TIL_EN_Pin
                          |PAN_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TIL_DIR_Pin|PAN_DIR_Pin|RIGHT_LED_R_Pin|RIGHT_LED_G_Pin
                          |RIGHT_LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_RESET_Pin|OLED_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : TOUCH_92_Pin TOUCH_84_Pin TOUCH_86_Pin TOUCH_91_Pin
                           TOUCH_NC_Pin TOUCH_85_Pin TOUCH_90_Pin */
  GPIO_InitStruct.Pin = TOUCH_92_Pin|TOUCH_84_Pin|TOUCH_86_Pin|TOUCH_91_Pin
                          |TOUCH_NC_Pin|TOUCH_85_Pin|TOUCH_90_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_Pin HEAD_LED_EN_Pin OneWIRE_DS18820_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin|HEAD_LED_EN_Pin|OneWIRE_DS18820_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN1_SENS_Pin */
  GPIO_InitStruct.Pin = PAN1_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PAN1_SENS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAN2_SENS_Pin TIL1_SENS_Pin TIL2_SENS_Pin */
  GPIO_InitStruct.Pin = PAN2_SENS_Pin|TIL1_SENS_Pin|TIL2_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TOP_LED_Pin PAD_POWER_EN_Pin */
  GPIO_InitStruct.Pin = TOP_LED_Pin|PAD_POWER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : TOP_BUTTON_Pin */
  GPIO_InitStruct.Pin = TOP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOP_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin OLED_R_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|OLED_R_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_3D_POW_EN_Pin USB_EN_Pin */
  GPIO_InitStruct.Pin = CAM_3D_POW_EN_Pin|USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_LED_R_Pin LEFT_LED_G_Pin LEFT_LED_B_Pin TIL_EN_Pin
                           PAN_EN_Pin */
  GPIO_InitStruct.Pin = LEFT_LED_R_Pin|LEFT_LED_G_Pin|LEFT_LED_B_Pin|TIL_EN_Pin
                          |PAN_EN_Pin;
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

  /*Configure GPIO pins : TIL_DIR_Pin PAN_DIR_Pin RIGHT_LED_R_Pin RIGHT_LED_G_Pin
                           RIGHT_LED_B_Pin OLED_RESET_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = TIL_DIR_Pin|PAN_DIR_Pin|RIGHT_LED_R_Pin|RIGHT_LED_G_Pin
                          |RIGHT_LED_B_Pin|OLED_RESET_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_83_Pin */
  GPIO_InitStruct.Pin = TOUCH_83_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_83_GPIO_Port, &GPIO_InitStruct);

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
