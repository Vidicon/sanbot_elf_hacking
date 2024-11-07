#include "Soft_I2C.h"
#include "stm32f2xx_hal.h"
#include <main.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Define the GPIO pins used for I2C
//------------------------------------------------------------------------------
#define SCL_PIN    SCL_Distance_J26_Pin
#define SDA_PIN    SDA_Distance_J26_Pin
#define I2C_PORT   GPIOC

//------------------------------------------------------------------------------
// Do some pre-calculations to speed up the I2C cycle.
//------------------------------------------------------------------------------
#define SDA_PIN_POS  SDA_PIN >> 1		// Calculate pin position
#define SDA_PIN_POS_SHIFT_INPUT  (0x03 << (2 * SDA_PIN_POS))
#define SDA_PIN_POS_SHIFT_OUTPUT (0x01 << (2 * SDA_PIN_POS))

#define Soft_I2C_DELAY()   delay_us(1)

//------------------------------------------------------------------------------
// Fast GPIO macros using BSRR register
// Is much faster than using the HAL libraries
//------------------------------------------------------------------------------
#define Soft_I2C_SCL_High()  (I2C_PORT->BSRR = SCL_PIN)
#define Soft_I2C_SCL_Low()   (I2C_PORT->BSRR = (SCL_PIN << 16))
#define Soft_I2C_SDA_High()  (I2C_PORT->BSRR = SDA_PIN)
#define Soft_I2C_SDA_Low()   (I2C_PORT->BSRR = (SDA_PIN << 16))

//------------------------------------------------------------------------------
// SDA Input and Output Mode Setup
// Precalculated values to prevent costly shift operation every time
//------------------------------------------------------------------------------
void Soft_I2C_SDA_Input() {
    I2C_PORT->MODER &= ~(SDA_PIN_POS_SHIFT_INPUT);  // Set to input mode
}

void Soft_I2C_SDA_Output() {
    I2C_PORT->MODER |= (SDA_PIN_POS_SHIFT_OUTPUT);  // Set to output mode
}

//------------------------------------------------------------------------------
// Low level read to speed up
//------------------------------------------------------------------------------
uint8_t Soft_I2C_Read_SDA() {
	return (I2C_PORT->IDR & (1 << SDA_PIN_POS)) ? 1 : 0;
//	return HAL_GPIO_ReadPin(SDA_Distance_J26_GPIO_Port, SDA_Distance_J26_Pin);
}

//------------------------------------------------------------------------------
// No real delay needed.
// The function call is enough?
//------------------------------------------------------------------------------
void delay_us(uint32_t us)
{

}

//------------------------------------------------------------------------------
// I2C Start Condition
//------------------------------------------------------------------------------
void Soft_I2C_Start() {
	Soft_I2C_SDA_Output();

	// Should already be high when we start.
	// Just to be sure.
	Soft_I2C_SDA_High();
	Soft_I2C_SCL_High();

	Soft_I2C_DELAY();
	Soft_I2C_SDA_Low();
	Soft_I2C_DELAY();
	Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// I2C Stop Condition
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// Read a byte from the I2C bus
//------------------------------------------------------------------------------
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    Soft_I2C_SDA_Input();

    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | Soft_I2C_ReadBit();
    }

    // Send ACK or NACK bit
    Soft_I2C_SDA_Output();
    Soft_I2C_WriteBit(!ack);  // Acknowledge if ack == 1 (is LOW), otherwise NACK
    return byte;
}

//------------------------------------------------------------------------------
// Write a byte to the I2C bus
//------------------------------------------------------------------------------
uint8_t Soft_I2C_WriteByte(uint8_t byte)
{
	Soft_I2C_SDA_Output();

    for (int i = 0; i < 8; i++)
    {
    	Soft_I2C_WriteBit(byte & 0x80);  // Write MSB first
        byte <<= 1;
    }

    // Read ACK/NACK bit
    Soft_I2C_SDA_Input();
    return Soft_I2C_ReadBit();
}

//------------------------------------------------------------------------------
// Write a bit to the I2C bus
//------------------------------------------------------------------------------
void Soft_I2C_WriteBit(uint8_t bit)
{
    if (bit) {
    	Soft_I2C_SDA_High();
    } else {
    	Soft_I2C_SDA_Low();
    }

    // Wait for data to be set before setting clock
    Soft_I2C_DELAY();

    // Clock pulse
    Soft_I2C_SCL_High();
    Soft_I2C_DELAY();
    Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// Read a bit from the I2C bus
//------------------------------------------------------------------------------
uint8_t Soft_I2C_ReadBit()
{
    uint8_t bit;

    Soft_I2C_SCL_High();
    Soft_I2C_DELAY();

    bit = Soft_I2C_Read_SDA();

    Soft_I2C_SCL_Low();

    return bit;
}

//------------------------------------------------------------------------------
// Functions to call from outside this file
//------------------------------------------------------------------------------
void Soft_I2C_Init()
{

}

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

    Soft_I2C_Start();

	Soft_I2C_WriteByte((slave_address << 1) + 1);

    data1 = Soft_I2C_ReadByte(1);  // Read byte
    data2 = Soft_I2C_ReadByte(0);  // Read byte

    Soft_I2C_Stop();

    return (int)(data1 << 8) + data2;
}

void Soft_I2C_Configure_Pins(uint16_t in_SDA_pin, uint16_t in_SLC_pin, uint16_t in_I2C_Port)
{

}

