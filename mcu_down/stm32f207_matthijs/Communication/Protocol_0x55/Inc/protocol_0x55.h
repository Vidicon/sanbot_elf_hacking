#ifndef _MYLIB_0x55_MESSAGES
#define _MYLIB_0x55_MESSAGES

#include "stm32f2xx.h"
#include "Encoders.h"

#define RESPONSE_TRUE 		1
#define RESPONSE_FALSE 		0

#define FIFO_RXSIZE			256

#define CMD_VERSION     0x01
#define CMD_LA_COLOR    0x10
#define CMD_RA_COLOR    0x11
#define CMD_BASE_COLOR  0x12

#define CMD_GET_ENCODERS 0x20
#define CMD_LA_MOVE		 0x30
#define CMD_RA_MOVE		 0x31

#define RESP_BIT 		0x80


struct PROTOCOL_0X55_Data_Type {
			uint8_t FIFO_Data[FIFO_RXSIZE+1];
			int BytesInBuffer;
			int TotalMsgSize;
		};

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetRxPointer();

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetTxPointer();

void Protocol_0x55_Init();

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

char Protocol_0x55_GetData(int Index);

void Protocol_0x55_ClearRxBuffer();

void SendEncoders(struct Encoders_Data_Type *EncoderData);

void Protocol_0x55_SendEncoders(char *Buffer, struct Encoders_Data_Type *EncoderData);

#endif
