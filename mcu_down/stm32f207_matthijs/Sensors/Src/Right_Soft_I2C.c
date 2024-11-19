#include "stm32f2xx_hal.h"
#include <main.h>
#include <Right_Soft_I2C.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Define the GPIO pins used for I2C
//------------------------------------------------------------------------------
//#define RIGHT_SCL_PIN    SCL_Distance_J26_Pin
//#define RIGHT_SDA_PIN    SDA_Distance_J26_Pin
//

#define RIGHT_SCL_PIN    0
#define RIGHT_SDA_PIN    1
#define RIGHT_I2C_PORT   GPIOC

////------------------------------------------------------------------------------
//// Do some pre-calculations to speed up the I2C cycle.
////------------------------------------------------------------------------------
//#define RIGHT_SDA_PIN_POS  RIGHT_SDA_PIN >> 1		// Calculate pin position
//#define RIGHT_SDA_PIN_POS_SHIFT_INPUT  (0x03 << (2 * RIGHT_SDA_PIN_POS))
//#define RIGHT_SDA_PIN_POS_SHIFT_OUTPUT (0x01 << (2 * RIGHT_SDA_PIN_POS))

#define Right_Soft_I2C_DELAY()   Right_Delay_us(1)

//------------------------------------------------------------------------------
// Fast GPIO macros using BSRR register
// Is much faster than using the HAL libraries
//------------------------------------------------------------------------------
#define Right_Soft_I2C_SCL_High()  (RIGHT_I2C_PORT->BSRR = (1 << RIGHT_SCL_PIN))
#define Right_Soft_I2C_SCL_Low()   (RIGHT_I2C_PORT->BSRR = ((1 << RIGHT_SCL_PIN) << 16))

#define Right_Soft_I2C_SDA_High()  (RIGHT_I2C_PORT->BSRR = (1 << RIGHT_SDA_PIN))
#define Right_Soft_I2C_SDA_Low()   (RIGHT_I2C_PORT->BSRR = ((1 << RIGHT_SDA_PIN) << 16))

//------------------------------------------------------------------------------
// SDA Input and Output Mode Setup
// Precalculated values to prevent costly shift operation every time
//------------------------------------------------------------------------------
void Right_Soft_I2C_SDA_Input() {
	RIGHT_I2C_PORT->MODER &= ~(0b11 << (RIGHT_SDA_PIN * 2)); // Clear bits 19 and 18 to set PB9 to input mode
}

void Right_Soft_I2C_SDA_Output() {
	RIGHT_I2C_PORT->MODER |= (0b01 << (RIGHT_SDA_PIN * 2));  // Set bits 19 and 18 to 01 for output mode
}

//------------------------------------------------------------------------------
// Low level read to speed up
//------------------------------------------------------------------------------
uint8_t Right_Soft_I2C_Read_SDA() {
	//	return HAL_GPIO_ReadPin(SDA_Distance_J26_GPIO_Port, SDA_Distance_J26_Pin);
	return (RIGHT_I2C_PORT->IDR & (1 << RIGHT_SDA_PIN)) ? 1 : 0;
}

//------------------------------------------------------------------------------
// No real delay needed.
// The function call is enough?
//------------------------------------------------------------------------------
void Right_Delay_us(uint32_t us)
{
	// Nothing here. Jump seems to take long enough to create a delay
}

//------------------------------------------------------------------------------
// I2C Start Condition
//------------------------------------------------------------------------------
 void Right_Soft_I2C_Start() {
	 Right_Soft_I2C_SDA_Output();

	// Should already be high when we start.
	// Just to be sure.
	Right_Soft_I2C_SDA_High();
	Right_Soft_I2C_SCL_High();

	Right_Soft_I2C_DELAY();
	Right_Soft_I2C_SDA_Low();
	Right_Soft_I2C_DELAY();
	Right_Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// I2C Stop Condition
//------------------------------------------------------------------------------
void Right_Soft_I2C_Stop() {
	Right_Soft_I2C_SDA_Output();

	Right_Soft_I2C_SCL_Low();
	Right_Soft_I2C_SDA_Low();
	Right_Soft_I2C_DELAY();
	Right_Soft_I2C_SCL_High();
	Right_Soft_I2C_DELAY();
	Right_Soft_I2C_SDA_High();

	Right_Soft_I2C_DELAY();
}

//------------------------------------------------------------------------------
// Read a byte from the I2C bus
//------------------------------------------------------------------------------
uint8_t Right_Soft_I2C_ReadByte(uint8_t ack)
{
	Right_Soft_I2C_SDA_Input();

    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | Right_Soft_I2C_ReadBit();
    }

    // Send ACK or NACK bit
    Right_Soft_I2C_SDA_Output();
    Right_Soft_I2C_WriteBit(!ack);  // Acknowledge if ack == 1 (is LOW), otherwise NACK
    return byte;
}

//------------------------------------------------------------------------------
// Write a byte to the I2C bus
//------------------------------------------------------------------------------
uint8_t Right_Soft_I2C_WriteByte(uint8_t byte)
{
	Right_Soft_I2C_SDA_Output();

    for (int i = 0; i < 8; i++)
    {
    	Right_Soft_I2C_WriteBit(byte & 0x80);  // Write MSB first
        byte <<= 1;
    }

    // Read ACK/NACK bit
    Right_Soft_I2C_SDA_Input();
    return Right_Soft_I2C_ReadBit();
}

//------------------------------------------------------------------------------
// Write a bit to the I2C bus
//------------------------------------------------------------------------------
void Right_Soft_I2C_WriteBit(uint8_t bit)
{
    if (bit) {
    	Right_Soft_I2C_SDA_High();
    } else {
    	Right_Soft_I2C_SDA_Low();
    }

    // Wait for data to be set before setting clock
    Right_Soft_I2C_DELAY();

    // Clock pulse
    Right_Soft_I2C_SCL_High();
    Right_Soft_I2C_DELAY();
    Right_Soft_I2C_SCL_Low();
}

//------------------------------------------------------------------------------
// Read a bit from the I2C bus
//------------------------------------------------------------------------------
uint8_t Right_Soft_I2C_ReadBit()
{
    uint8_t bit;

    Right_Soft_I2C_SCL_High();
    Right_Soft_I2C_DELAY();

    bit = Right_Soft_I2C_Read_SDA();

    Right_Soft_I2C_SCL_Low();

    return bit;
}

//------------------------------------------------------------------------------
// Functions to call from outside this file
//------------------------------------------------------------------------------
void Right_Soft_I2C_Write(uint8_t slave_address, uint8_t data)
{
	Right_Soft_I2C_Start();
	Right_Soft_I2C_WriteByte(slave_address << 1);
	Right_Soft_I2C_WriteByte(data);
	Right_Soft_I2C_Stop();
}

int Right_Soft_I2C_Read(uint8_t slave_address)
{
    uint8_t data1;
    uint8_t data2;

    Right_Soft_I2C_Start();

    Right_Soft_I2C_WriteByte((slave_address << 1) + 1);

    data1 = Right_Soft_I2C_ReadByte(1);  // Read byte
    data2 = Right_Soft_I2C_ReadByte(0);  // Read byte

    Right_Soft_I2C_Stop();

    return (int)(data1 << 8) + data2;
}


