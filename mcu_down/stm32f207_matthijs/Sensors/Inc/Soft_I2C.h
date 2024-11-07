/*
 * Soft_I2C.h
 *
 *  Created on: Oct 31, 2024
 *      Author: matthijs
 */

#ifndef INC_SOFT_I2C_H_
#define INC_SOFT_I2C_H_

#include "stm32f2xx.h"

void Soft_I2C_Init();

void Soft_I2C_Start();

void Soft_I2C_Stop();

//void Soft_I2C_WriteByte(uint8_t byte);
uint8_t Soft_I2C_WriteByte(uint8_t byte);

uint8_t Soft_I2C_ReadBit();

void Soft_I2C_WriteBit(uint8_t bit);

void Soft_I2C_Write(uint8_t slave_address, uint8_t data);

int Soft_I2C_Read(uint8_t slave_address);

uint8_t Soft_I2C_ReadByte(uint8_t ack);

//uint8_t Soft_I2C_Read(uint8_t slave_address);
int Soft_I2C_Read(uint8_t slave_address);

void Soft_I2C_Configure_Pins(uint16_t in_SDA_pin, uint16_t in_SLC_pin, uint16_t in_I2C_Port);

#endif /* INC_SOFT_I2C_H_ */
