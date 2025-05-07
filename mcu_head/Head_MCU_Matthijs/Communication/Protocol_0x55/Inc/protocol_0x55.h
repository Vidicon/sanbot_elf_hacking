#ifndef _MYLIB_0x55_MESSAGES
#define _MYLIB_0x55_MESSAGES

#include "stm32f1xx.h"

#define RESPONSE_TRUE 		1
#define RESPONSE_FALSE 		0

#define FIFO_SIZE			64

#define CMD_VERSION     		0x01
#define CMD_LA_COLOR    		0x10
#define CMD_RA_COLOR    		0x11
#define CMD_BASE_COLOR  		0x12
#define CMD_BA_COLOR    		0x13
#define CMD_LARA_COLOR			0x14

#define CMD_LEFTHEAD_COLOR 		0x15
#define CMD_RIGHTHEAD_COLOR 	0x16

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


#define CMD_VERSION_HEAD 		0x40
#define CMD_HEAD_PAN_MOVE 		0x41
#define CMD_HEAD_PAN_BRAKE 		0x42
#define CMD_HEAD_TILT_MOVE 		0x43
#define CMD_HEAD_TILT_BRAKE 	0x44
#define CMD_HEAD_LEFT_COLOR 	0x45
#define CMD_HEAD_RIGHT_COLOR 	0x46
#define CMD_HEAD_TILT_HOME 		0x47
#define CMD_HEAD_PAN_HOME 		0x48
#define CMD_HEAD_EYES 			0x49
#define CMD_HEAD_LAMP 			0x4A
#define CMD_HEAD_STOP 			0x4B
#define CMD_HEAD_LAST 			0x4F

#define RESP_BIT 				0x80

// Commands to body MCU
#define HEAD_BUTTON_PRESSED		0x01
#define HEAD_BUTTON_RELEASED	0x02


struct PROTOCOL_0X55_Data_Type {
			uint8_t FIFO_Data[FIFO_SIZE+1];
			int BytesInBuffer;
			int TotalMsgSize;
		};

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetRxPointer();

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetTxPointer();

void Protocol_0x55_Init();

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


#endif
