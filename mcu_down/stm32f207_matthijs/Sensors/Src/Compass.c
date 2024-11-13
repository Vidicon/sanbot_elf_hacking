/*
 * Compass.h
 *
 *  Created on: Nov 13, 2024
 *      Author: matthijs
 */
#include "stm32f2xx_hal.h"
#include "Compass.h"
#include <string.h>

struct Compass_Sensor_Type CompassData;

I2C_HandleTypeDef *Compass_I2C;
uint8_t Compass_Address = 0x3c;
HAL_StatusTypeDef status;

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
void Compass_Init(I2C_HandleTypeDef *hI2C)
{
	Compass_I2C = hI2C;

	__HAL_RCC_GPIOC_CLK_ENABLE();  // Assuming I2C3 is on GPIOC pins
	__HAL_RCC_I2C3_CLK_ENABLE();

	// FUNC_CTRL_A (3Fh)
	// Module on
	uint8_t cmd;
	cmd = 0b00100000;

	cmd = 0b00000000;  // ???
//	status = HAL_I2C_Mem_Write(Compass_I2C, Compass_Address, 0x3f, I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);


	// CFG_REG_A_M (60h)
	cmd = 0b00000000;
	status = HAL_I2C_Mem_Write(Compass_I2C, Compass_Address, 0x60, I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);

	// Hard iron offset
	//	OFFSET_X_REG_L_M (45h) and OFFSET_X_REG_H_M (46h)

	uint8_t zeros[6];
	memset(zeros, 0, 6);

	status = HAL_I2C_Mem_Write(Compass_I2C, Compass_Address, 0x45, I2C_MEMADD_SIZE_8BIT, &zeros[0], 6, HAL_MAX_DELAY);



//	(Compass_I2C, 0x3C, 0x60, I2C_MEMADD_SIZE_8BIT, data1, 6, HAL_MAX_DELAY);
}


void Compass_Update()
{
	// Write to device and sets read pointer
	// Reads the data

	// 0x3C is correct.

	//  Configuration registers
	//  CFG_REG_A_M 0x60	--> 3	--> OK
	//  CFG_REG_B_M 0x61	--> 0	--> OK
	//  CFG_REG_C_M 0x61	--> 0 	--> OK
	//  status = HAL_I2C_Mem_Read(Compass_I2C, 0x3C, 0x60, I2C_MEMADD_SIZE_8BIT, data1, 6, HAL_MAX_DELAY);

	//  0x4F --> Who I am register --> Returns 0x40 --> OK
	//	status = HAL_I2C_Mem_Read(Compass_I2C, 0x3C, 0x4F, I2C_MEMADD_SIZE_8BIT, data1, 1, HAL_MAX_DELAY);

	//  Status register
	//  STATUS_REG_M 0x67	--> 0	--> NO new data is available.
	//	status = HAL_I2C_Mem_Read(Compass_I2C, 0x3C, 0x67, I2C_MEMADD_SIZE_8BIT, data1, 1, HAL_MAX_DELAY);

	status = HAL_I2C_Mem_Read(Compass_I2C, 0x3C, 0x68, I2C_MEMADD_SIZE_8BIT, &CompassData.Raw[0], 6, HAL_MAX_DELAY);

//	if (status == 0)
	{
//		CompassData.X = (int)(CompassData.Raw[0] + (CompassData.Raw[1] << 8));
//		CompassData.Y = (int)(CompassData.Raw[2] + (CompassData.Raw[3] << 8));
//		CompassData.Z = CompassData.Raw[4] + (CompassData.Raw[5] << 8);

		CompassData.X = (int)(CompassData.Raw[0] + (CompassData.Raw[1] << 8));
		CompassData.Y = (int)(CompassData.Raw[2] + (CompassData.Raw[3] << 8));
		CompassData.Z = (int)(CompassData.Raw[4] + (CompassData.Raw[5] << 8));
	}


	HAL_Delay(1);
}

