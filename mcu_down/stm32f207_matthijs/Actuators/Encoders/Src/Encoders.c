#include "Encoders.h"
#include "stm32f2xx.h"
#include <main.h>
#include <stdio.h>
#include <string.h>

#define ENCODER_RX_BUFFER_SIZE 15
uint8_t Encoder_Raw_Buffer[ENCODER_RX_BUFFER_SIZE];

struct Encoders_Data_Type EncoderData;

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void Encoders_Init(UART_HandleTypeDef *huart)
{
  /* Start UART reception in DMA mode */
  memset(&Encoder_Raw_Buffer[0], 0, ENCODER_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(huart, Encoder_Raw_Buffer, ENCODER_RX_BUFFER_SIZE);

//	LeftArm_State.Angle = 0;
//	LeftArm_State.Direction = Arm_Up;
//	LeftArm_State.Homed = NotHomed;
//	LeftArm_State.MotionState = Motion_Disabled;
//	LeftArm_State.Timer = 0;
//	LeftArm_State.SelTestRunning = 0;
//	LeftArm_State.TIM = htim;
//	LeftArm_State.Speed = 0;


//	HAL_TIM_Base_Start(htim);
//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void Encoders_SelfTest()
{
//	// Force a home sequence
//	LeftArm_EnableBrake(False);
//
//	LeftArm_State.SelTestRunning = 1;
//	LeftArm_State.MotionState = Motion_MovingUp;
//	LeftArm_State.Speed = 10;
}

/* DMA Transfer Complete Interrupt Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)
    {
        /* Handle the reception of data here */
        /* Reset DMA reception if needed */

    	for (int i=0; i < 5; i++)
    	{
			// Negative values have 0x80 is bytes 5, 7, 9, 11, 13
    		if (Encoder_Raw_Buffer[i*2 + 5] == 0x80)
    		{
    			EncoderData.Encoder[i] -= Encoder_Raw_Buffer[i*2+4];
    		}
    		else
    		{
    			EncoderData.Encoder[i] += Encoder_Raw_Buffer[i*2+4];
    		}
    	}

		memset(&Encoder_Raw_Buffer[0], 0, ENCODER_RX_BUFFER_SIZE);
		HAL_UART_Receive_DMA(huart, Encoder_Raw_Buffer, ENCODER_RX_BUFFER_SIZE);
    }
}

