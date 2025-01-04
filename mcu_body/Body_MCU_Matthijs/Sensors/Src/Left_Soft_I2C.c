#include "stm32f2xx_hal.h"
#include <main.h>
#include <Left_Soft_I2C.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Define the GPIO pins used for I2C
//------------------------------------------------------------------------------
#define LEFT_SCL_PIN    8		// 0x0100
#define LEFT_SDA_PIN    9		// 0x0200
#define LEFT_I2C_PORT   GPIOB

#define Left_Soft_I2C_DELAY()   Left_Delay_us(1)

//------------------------------------------------------------------------------
// Fast GPIO macros using BSRR register
// Is much faster than using the HAL libraries
//------------------------------------------------------------------------------
#define Left_Soft_I2C_SCL_High()  (LEFT_I2C_PORT->BSRR = (1 << LEFT_SCL_PIN))
#define Left_Soft_I2C_SCL_Low()   (LEFT_I2C_PORT->BSRR = ((1 << LEFT_SCL_PIN) << 16))

#define Left_Soft_I2C_SDA_High()  (LEFT_I2C_PORT->BSRR = (1 << LEFT_SDA_PIN))
#define Left_Soft_I2C_SDA_Low()   (LEFT_I2C_PORT->BSRR = ((1 << LEFT_SDA_PIN) << 16))

//------------------------------------------------------------------------------
// SDA Input and Output Mode Setup
// Precalculated values to prevent costly shift operation every time
//------------------------------------------------------------------------------
void Left_Soft_I2C_SDA_Input() {
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LEFT_I2C_PORT->MODER &= ~(0b11 << (LEFT_SDA_PIN * 2));
}

void Left_Soft_I2C_SDA_Output() {
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output mode
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LEFT_I2C_PORT->MODER |= (0b01 << (LEFT_SDA_PIN * 2));
}

//------------------------------------------------------------------------------
// Low level read to speed up
//------------------------------------------------------------------------------
uint8_t Left_Soft_I2C_Read_SDA() {
//	return HAL_GPIO_ReadPin(SDA_Distance_J18_GPIO_Port, SDA_Distance_J18_Pin);
	return (LEFT_I2C_PORT->IDR & (1 << LEFT_SDA_PIN)) ? 1 : 0;
}

//------------------------------------------------------------------------------
// No real delay needed.
// The function call is enough?
//------------------------------------------------------------------------------
void Left_Delay_us(uint32_t us)
{
	// Nothing here. Jump seems to take long enough to create a delay
}

//------------------------------------------------------------------------------
// I2C Start Condition
//------------------------------------------------------------------------------
 void Left_Soft_I2C_Start() {
	Left_Soft_I2C_SDA_Output();

	// Should already be high when we start.
	// Just to be sure.
	Left_Soft_I2C_SDA_High();
	Left_Soft_I2C_SCL_High();

	Left_Soft_I2C_DELAY();
	Left_Soft_I2C_SDA_Low();
	Left_Soft_I2C_DELAY();
	Left_Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// I2C Stop Condition
//------------------------------------------------------------------------------
void Left_Soft_I2C_Stop() {
	Left_Soft_I2C_SDA_Output();

	Left_Soft_I2C_SCL_Low();
	Left_Soft_I2C_SDA_Low();
	Left_Soft_I2C_DELAY();
	Left_Soft_I2C_SCL_High();
	Left_Soft_I2C_DELAY();
	Left_Soft_I2C_SDA_High();

	Left_Soft_I2C_DELAY();
}

//------------------------------------------------------------------------------
// Read a byte from the I2C bus
//------------------------------------------------------------------------------
uint8_t Left_Soft_I2C_ReadByte(uint8_t ack)
{
	Left_Soft_I2C_SDA_Input();

    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | Left_Soft_I2C_ReadBit();
    }

    // Send ACK or NACK bit
    Left_Soft_I2C_SDA_Output();
    Left_Soft_I2C_WriteBit(!ack);  // Acknowledge if ack == 1 (is LOW), otherwise NACK
    return byte;
}

//------------------------------------------------------------------------------
// Write a byte to the I2C bus
//------------------------------------------------------------------------------
uint8_t Left_Soft_I2C_WriteByte(uint8_t byte)
{
	Left_Soft_I2C_SDA_Output();

    for (int i = 0; i < 8; i++)
    {
    	Left_Soft_I2C_WriteBit(byte & 0x80);  // Write MSB first
        byte <<= 1;
    }

    // Read ACK/NACK bit
    Left_Soft_I2C_SDA_Input();
    return Left_Soft_I2C_ReadBit();
}

//------------------------------------------------------------------------------
// Write a bit to the I2C bus
//------------------------------------------------------------------------------
void Left_Soft_I2C_WriteBit(uint8_t bit)
{
    if (bit) {
    	Left_Soft_I2C_SDA_High();
    } else {
    	Left_Soft_I2C_SDA_Low();
    }

    // Wait for data to be set before setting clock
    Left_Soft_I2C_DELAY();

    // Clock pulse
    Left_Soft_I2C_SCL_High();
    Left_Soft_I2C_DELAY();
    Left_Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// Read a bit from the I2C bus
//------------------------------------------------------------------------------
uint8_t Left_Soft_I2C_ReadBit()
{
    uint8_t bit;

    Left_Soft_I2C_SCL_High();
    Left_Soft_I2C_DELAY();

    bit = Left_Soft_I2C_Read_SDA();

    Left_Soft_I2C_SCL_Low();

    return bit;
}

//------------------------------------------------------------------------------
// Functions to call from outside this file
//------------------------------------------------------------------------------
void Left_Soft_I2C_Write(uint8_t slave_address, uint8_t data)
{
	Left_Soft_I2C_Start();
	Left_Soft_I2C_WriteByte(slave_address << 1);
	Left_Soft_I2C_WriteByte(data);
	Left_Soft_I2C_Stop();
}

int Left_Soft_I2C_Read(uint8_t slave_address)
{
    uint8_t data1;
    uint8_t data2;

    Left_Soft_I2C_Start();

    Left_Soft_I2C_WriteByte((slave_address << 1) + 1);

    data1 = Left_Soft_I2C_ReadByte(1);  // Read byte
    data2 = Left_Soft_I2C_ReadByte(0);  // Read byte

    Left_Soft_I2C_Stop();

    return (int)(data1 << 8) + data2;
}


