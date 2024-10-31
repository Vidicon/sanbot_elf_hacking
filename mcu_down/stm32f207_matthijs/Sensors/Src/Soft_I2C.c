/*
 * Soft_I2C.c
 *
 *  Created on: Oct 31, 2024
 *      Author: matthijs
 */

#include "Soft_I2C.h"
#include "stm32f2xx_hal.h"
#include <main.h>
#include <stdio.h>

// Define the GPIO pins used for I2C
#define SCL_PIN    SCL_Distance_J26_Pin
#define SDA_PIN    SDA_Distance_J26_Pin
#define I2C_PORT   GPIOC


//#define Soft_I2C_DELAY()   HAL_Delay(1)  // Simple delay for I2C timing (1ms for slow clock)
#define Soft_I2C_DELAY()   delay_us(1)

// Functions to control the SCL and SDA lines
void Soft_I2C_SCL_High() { HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); }
void Soft_I2C_SCL_Low()  { HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); }
void Soft_I2C_SDA_High() { HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); }
void Soft_I2C_SDA_Low()  { HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); }

void Soft_I2C_SDA_Input()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin : SDA_Distance_J26_Pin */
	GPIO_InitStruct.Pin = SDA_Distance_J26_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SDA_Distance_J26_GPIO_Port, &GPIO_InitStruct);
}

//void Soft_I2C_SDA_Input() { GPIO_InitTypeDef GPIO_InitStruct = {0}; \
//                        GPIO_InitStruct.Pin = SDA_PIN; \
//                        GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
//                        GPIO_InitStruct.Pull = GPIO_NOPULL; \
//                        HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct); }

void Soft_I2C_SDA_Output()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin : SDA_Distance_J26_Pin */
	GPIO_InitStruct.Pin = SDA_Distance_J26_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SDA_Distance_J26_GPIO_Port, &GPIO_InitStruct);
}

//void Soft_I2C_SDA_Output() { GPIO_InitTypeDef GPIO_InitStruct = {0}; \
//                         GPIO_InitStruct.Pin = SDA_PIN; \
//                         GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; \
//                         GPIO_InitStruct.Pull = GPIO_NOPULL; \
//                         HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct); }

uint8_t Soft_I2C_Read_SDA() {
    return HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN);
}


#include "stm32f2xx_hal.h"

void delay_us(uint32_t us) {
    // Get the current TIM2 counter value
    uint32_t start = TIM9->CNT;
    // Wait until the time elapsed
    while ((TIM9->CNT - start) < us);
}

// I2C Start Condition
void Soft_I2C_Start() {
	Soft_I2C_SDA_Output();

	Soft_I2C_SDA_High();
	Soft_I2C_SCL_High();
	Soft_I2C_DELAY();
	Soft_I2C_SDA_Low();
	Soft_I2C_DELAY();
	Soft_I2C_SCL_Low();

	Soft_I2C_DELAY();
}

// I2C Stop Condition
void Soft_I2C_Stop() {
	Soft_I2C_SDA_Output();

	Soft_I2C_SCL_Low();
	Soft_I2C_SDA_Low();
	Soft_I2C_DELAY();
	Soft_I2C_SCL_High();
	Soft_I2C_DELAY();
	Soft_I2C_SDA_High();

	Soft_I2C_DELAY();
}

// Read a byte from the I2C bus
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | Soft_I2C_ReadBit();
    }

    // Send ACK or NACK bit
    Soft_I2C_WriteBit(!ack);  // Acknowledge if ack == 1 (is LOW), otherwise NACK
    return byte;
}

// Write a byte to the I2C bus
uint8_t Soft_I2C_WriteByte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
    	Soft_I2C_WriteBit(byte & 0x80);  // Write MSB first
        byte <<= 1;
    }

    // Read ACK/NACK bit
    return Soft_I2C_ReadBit();
}

// Write a bit to the I2C bus
void Soft_I2C_WriteBit(uint8_t bit)
{
	Soft_I2C_SDA_Output();

    if (bit) {
    	Soft_I2C_SDA_High();
    } else {
    	Soft_I2C_SDA_Low();
    }
    Soft_I2C_DELAY();
    Soft_I2C_SCL_High();
    Soft_I2C_DELAY();
    Soft_I2C_SCL_Low();
}

// Read a bit from the I2C bus
uint8_t Soft_I2C_ReadBit()
{
    uint8_t bit;

    Soft_I2C_SDA_Input();
    Soft_I2C_DELAY();

    Soft_I2C_SCL_High();
    Soft_I2C_DELAY();

    bit = Soft_I2C_Read_SDA();

    Soft_I2C_SCL_Low();
    Soft_I2C_DELAY();

    return bit;
}

// Example function to write data to a slave device
void Soft_I2C_Write(uint8_t slave_address, uint8_t data)
{
	Soft_I2C_Start();
	Soft_I2C_WriteByte(slave_address << 1);
	Soft_I2C_WriteByte(data);
	Soft_I2C_Stop();
}

int Soft_I2C_Read(uint8_t slave_address)
{
    uint8_t data1;
    uint8_t data2;

    Soft_I2C_DELAY();
    Soft_I2C_DELAY();
    Soft_I2C_DELAY();
    Soft_I2C_DELAY();
    Soft_I2C_DELAY();

    Soft_I2C_Start();

	Soft_I2C_WriteByte((slave_address << 1) + 1);

    data1 = Soft_I2C_ReadByte(1);  // Read byte
    data2 = Soft_I2C_ReadByte(0);  // Read byte

    Soft_I2C_Stop();

    return (int)(data1 << 8) + data2;
}


//----------------------------------------------------------------
//
//----------------------------------------------------------------
void Soft_I2C_Init()
{
	// TO DO
}


