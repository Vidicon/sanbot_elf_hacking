#ifndef _MYLIB_0x55_MESSAGES
#define _MYLIB_0x55_MESSAGES

#include "stm32f2xx.h"
#include "Encoders.h"
#include "MotionSensors.h"
#include "DistanceSensors.h"
#include "Compass.h"
#include "Battery.h"

#define RESPONSE_TRUE 		1
#define RESPONSE_FALSE 		0

#define FIFO_SIZE			64

#define CMD_VERSION     		0x01
#define CMD_LA_COLOR    		0x10
#define CMD_RA_COLOR    		0x11
#define CMD_BASE_COLOR  		0x12
#define CMD_BA_COLOR    		0x13
#define CMD_LARA_COLOR			0x14

#define CMD_GET_ENCODERS		0x20
#define CMD_GET_MOTIONSENSORS  	0x21
#define CMD_GET_DISTANCESENSORS 0x22
#define CMD_GET_COMPASS 	    0x23
#define CMD_GET_BATTERY 	    0x24


#define CMD_LA_MOVE		 		0x30
#define CMD_RA_MOVE		 		0x31
#define CMD_BASE_MOVE	 		0x32
#define CMD_COMP_MOVE			0x33
#define CMD_BASE_BRAKE	 		0x34
#define CMD_LA_HOME 			0x35
#define CMD_RA_HOME 			0x36
#define CMD_BODY_STOP 			0x37
#define CMD_BODY_LAST 			0x3F

#define RESP_BIT 				0x80

struct PROTOCOL_0X55_Data_Type {
			uint8_t FIFO_Data[FIFO_SIZE+1];
			int BytesInBuffer;
			int TotalMsgSize;
		};

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetRxPointer();

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetTxPointer();

void Protocol_0x55_Init(UART_HandleTypeDef *huart);

void Protocol_0x55_NewData(uint8_t* Buf, uint32_t *Len);

uint8_t Protocol_0x55_CheckFifo();

void Protocol_0x55_MarkProcessed();

int Protocol_0x55_GetCommand();

void SendVersion(void);

void Protocol_0x55_SendVersion(char *Buffer);

void Protocol_0x55_PrepareNewMessage(char *Buffer, char Command, char Response);

void Protocol_0x55_SetLength(char *Buffer, uint8_t datalen);

void Protocol_0x55_AddCRC(char *Buffer, uint8_t datalen);

void Protocol_0x55_Send(char *data, uint8_t datalen);

uint16_t Protocol_0x55_CalculateCRC16(char *data, uint8_t msgSize);

signed char Protocol_0x55_GetData(int Index);

void Protocol_0x55_ClearRxBuffer();



void SendEncoders(struct Encoders_Data_Type *EncoderData);

void Protocol_0x55_SendEncoders(char *Buffer, struct Encoders_Data_Type *EncoderData);


void SendMotionSensors(struct MotionSensors_Data_Type *MotionSensors_State);

void Protocol_0x55_SendMotionEvent(char *Buffer, struct MotionSensors_Data_Type *MotionSensors_State);


void SendDistanceSensors(struct Distance_Sensor_Type *DistanceData);

void Protocol_0x55_SendDistanceEvent(char *Buffer, struct Distance_Sensor_Type *DistanceData);


void SendCompass(struct Compass_Sensor_Type *CompassData);

void Protocol_0x55_SendCompass(char *Buffer, struct Compass_Sensor_Type *CompassData);


void SendBattery(struct Battery_Sensor_Type *BatteryData);

void Protocol_0x55_SendBattery(char *Buffer, struct Battery_Sensor_Type *BatteryData);

//------------------------------------------------------------------------------------------
void SendCompassMoveDone(uint8_t Succes);

void Protocol_0x55_SendCompassMoveDone(char *Buffer, uint8_t Succes);


#endif
