/*
 * Battery.c
 *
 *  Created on: Nov 21, 2024
 *      Author: matthijs
 */
#include "stm32f2xx_hal.h"
#include <string.h>
#include "protocol_0x55.h"
#include "Battery.h"


struct Battery_Sensor_Type BatteryData;

I2C_HandleTypeDef *Battery_I2C;
uint8_t Battery_Address = 0x16;
HAL_StatusTypeDef status;

uint8_t reg;
uint8_t command[2];
uint8_t response[10];

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
struct Battery_Sensor_Type *Battery_GetPointer()
{
	return &BatteryData;
}

//----------------------------------------------------------------
// Return pointer instead of copy
//----------------------------------------------------------------
void Battery_Init(I2C_HandleTypeDef *hI2C)
{
	Battery_I2C = hI2C;

	__HAL_RCC_GPIOC_CLK_ENABLE();  // Assuming I2C3 is on GPIOC pins
	__HAL_RCC_I2C3_CLK_ENABLE();


	//----------------------------------------------------------------------------
	// ManufacturerAccess(0x00)
	// Device Type(0x0001)
	//----------------------------------------------------------------------------
  	reg = 0x00;
  	command[0] = 0x01;
  	command[1] = 0x00;

	status = HAL_I2C_Mem_Write(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &command[0], 2, HAL_MAX_DELAY);
	HAL_Delay(1);
	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.DeviceType = (uint16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.DeviceType = 0;
	}


	//----------------------------------------------------------------------------
	// ManufacturerAccess(0x00)
	// Firmware Version(0x0002)
	//----------------------------------------------------------------------------
  	reg = 0x00;
  	command[0] = 0x02;
  	command[1] = 0x00;

	status = HAL_I2C_Mem_Write(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &command[0], 2, HAL_MAX_DELAY);
	HAL_Delay(1);
	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.FW_Version = (uint16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.FW_Version  = 0;
	}

	//----------------------------------------------------------------------------
	// ManufacturerAccess(0x00)
	// Hardware Version(0x0003)
	//----------------------------------------------------------------------------
  	reg = 0x00;
  	command[0] = 0x03;
  	command[1] = 0x00;

	status = HAL_I2C_Mem_Write(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &command[0], 2, HAL_MAX_DELAY);
	HAL_Delay(1);
	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.HW_Version = (uint16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.HW_Version  = 0;
	}
}

void Battery_Update()
{
	//----------------------------------------------------------------------------
	// ManufacturerAccess(0x00)
  	// Device Type(0x0001)
  	// Manufacturer Status (0x0006)
	//----------------------------------------------------------------------------
  	reg = 0x00;
  	command[0] = 0x06;
  	command[1] = 0x00;

	status = HAL_I2C_Mem_Write(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &command[0], 2, HAL_MAX_DELAY);
	HAL_Delay(1);
	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.BatteryState = (uint16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.BatteryState  = 0;
	}

	//----------------------------------------------------------------------------
  	// 0X08 : Temperature
  	// LSB MSB
	//----------------------------------------------------------------------------
  	reg = 0x08;
  	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.Temperature = (int16_t)(response[1] << 8) + response[0] - 2730;
	}
	else
	{
		BatteryData.Temperature  = 0;
	}

	//----------------------------------------------------------------------------
  	// 0X09 : Voltage
  	// LSB MSB
	//----------------------------------------------------------------------------
  	reg = 0x09;
  	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

  	if (status == HAL_OK)
	{
  		BatteryData.Voltage = (int16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.Voltage  = 0;
	}

		//----------------------------------------------------------------------------
  	// 0X0A : Current
  	// LSB MSB
	//----------------------------------------------------------------------------
  	reg = 0x0A;
  	status = HAL_I2C_Mem_Read(Battery_I2C, Battery_Address, reg, I2C_MEMADD_SIZE_8BIT, &response[0], 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		BatteryData.Current = (int16_t)(response[1] << 8) + response[0];
	}
	else
	{
		BatteryData.Current  = 0;
	}

	SendBattery(Battery_GetPointer());
}
