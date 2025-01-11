#include "Encoders.h"
#include "stm32f2xx.h"
#include <main.h>
#include <stdio.h>
#include <string.h>

#define ENCODER_RX_BUFFER_SIZE 15
uint8_t Encoder_Raw_Buffer[ENCODER_RX_BUFFER_SIZE];

struct Encoders_Data_Type EncoderData;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct Encoders_Data_Type *Encoders_GetPointer()
{
	return &EncoderData;
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void Encoders_Init(UART_HandleTypeDef *huart)
{
	/* Start UART reception in DMA mode */
	memset(&Encoder_Raw_Buffer[0], 0, ENCODER_RX_BUFFER_SIZE);
	HAL_UART_Receive_DMA(huart, Encoder_Raw_Buffer, ENCODER_RX_BUFFER_SIZE);

	EncoderData.Encoder[0] = 0;
	EncoderData.Encoder[1] = 0;
	EncoderData.Encoder[2] = 0;
	EncoderData.Encoder[3] = 0;
	EncoderData.Encoder[4] = 0;

	EncoderData.NewData[0] = 0;
	EncoderData.NewData[1] = 0;
	EncoderData.NewData[2] = 0;
	EncoderData.NewData[3] = 0;
	EncoderData.NewData[4] = 0;

	EncoderData.RxCounter = 0;
}

void Encoders_SelfTest()
{

}

/* DMA Transfer Complete Interrupt Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int TmpEncoder = 0;

	if(huart->Instance == USART6)
    {
        /* Handle the reception of data here */
        /* Reset DMA reception if needed */
    	EncoderData.RxCounter += 1;

    	for (int i=0; i < 5; i++)
    	{
    		EncoderData.NewData[i] = 1;

    		// Negative values have 0x80 is bytes 5, 7, 9, 11, 13
    		if (Encoder_Raw_Buffer[i*2 + 5] == 0x80)
    		{
    			// Clear the 0x80 bit
    			Encoder_Raw_Buffer[i*2 + 5] = Encoder_Raw_Buffer[i*2 + 5] ^ 0x80;

    			TmpEncoder = (Encoder_Raw_Buffer[i*2 + 5] << 8) + Encoder_Raw_Buffer[i*2+4];

    			if (i == 3)
    			{
        			EncoderData.Encoder[i] += TmpEncoder;
    			}
    			else
    			{
    				EncoderData.Encoder[i] -= TmpEncoder;
    			}
    		}
    		else
    		{
    			TmpEncoder = (Encoder_Raw_Buffer[i*2 + 5] << 8) + Encoder_Raw_Buffer[i*2+4];

    			if (i == 3)
    			{
    				EncoderData.Encoder[i] -= TmpEncoder;
    			}
    			else
    			{
    				EncoderData.Encoder[i] += TmpEncoder;
    			}
    		}
    	}

		memset(&Encoder_Raw_Buffer[0], 0, ENCODER_RX_BUFFER_SIZE);
		HAL_UART_Receive_DMA(huart, Encoder_Raw_Buffer, ENCODER_RX_BUFFER_SIZE);
    }
}

