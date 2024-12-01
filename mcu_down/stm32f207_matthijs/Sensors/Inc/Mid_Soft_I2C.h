/*
 * Soft_I2C.h
 *
 *  Created on: Oct 31, 2024
 *      Author: matthijs
 */

#ifndef INC_MID_SOFT_I2C_H_
#define INC_MID_SOFT_I2C_H_

#include "stm32f2xx.h"

void Mid_Delay_us(uint32_t us);

void Mid_Soft_I2C_Start();

void Mid_Soft_I2C_Stop();

uint8_t Mid_Soft_I2C_WriteByte(uint8_t byte);

uint8_t Mid_Soft_I2C_ReadBit();

void Mid_Soft_I2C_WriteBit(uint8_t bit);

int Mid_Soft_I2C_Read(uint8_t slave_address);

uint8_t Mid_Soft_I2C_ReadByte(uint8_t ack);

int Mid_Soft_I2C_Read(uint8_t slave_address);

void Mid_Soft_I2C_Configure_Pins(uint16_t in_SDA_pin, uint16_t in_SLC_pin, uint16_t in_I2C_Port);


void Mid_Soft_I2C_Write(uint8_t slave_address, uint8_t data);

int Mid_Soft_I2C_Read(uint8_t slave_address);

#endif /* INC_MID_SOFT_I2C_H_ */
