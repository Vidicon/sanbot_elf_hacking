#ifndef _MYLIB_0x55_MESSAGES
#define _MYLIB_0x55_MESSAGES

#include "stm32f2xx.h"

#define RESPONSE_TRUE 		1
#define RESPONSE_FALSE 		0

#define FIFO_RXSIZE			1024

#define CMD_VERSION 		1


struct PROTOCOL_0X55_Data_Type {
			uint8_t FIFO_Data[FIFO_RXSIZE+1];
			uint8_t NewData;
		};

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetRxPointer();

struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetTxPointer();

void Protocol_0x55_Init();

uint8_t Protocol_0x55_CheckFifo();

void Protocol_0x55_ProcessRxCommand();

void Protocol_0x55_SendVersion(char *Buffer);

void Protocol_0x55_PrepareNewMessage(char *Buffer, char Command, char Response);

void Protocol_0x55_SetLength(char *Buffer, uint8_t datalen);

void Protocol_0x55_AddCRC(char *Buffer, uint8_t datalen);

void Protocol_0x55_Send(char *data, uint8_t datalen);

uint16_t Protocol_0x55_CalculateCRC16(char *data, uint8_t msgSize);



//void MSG_PrepareNewMessage(char* Buffer, char Command, char Response);
//
//void MSG_SetLength(char* Buffer, uint8_t datalen);
//
//void MSG_AddCRC(char* Buffer, uint8_t datalen);
//
//uint16_t MSG_CalculateCRC16(uint8_t *data, uint8_t datalen);
//
//void MSG_Send(char* Buffer, UART_HandleTypeDef Uart, uint8_t datalen);
//
//void MSG_AddHello(char* Buffer);
//
//void MSG_SendParam(char* Buffer, UART_HandleTypeDef Uart, uint8_t *ParamBuffer, uint8_t datalen);
//
//void MSG_SendVariables(char* Buffer, UART_HandleTypeDef Uart, uint8_t *VariablesBuffer, uint8_t datalen);
//
//void MSG_Response_OK(char* Buffer, UART_HandleTypeDef Uart, int Command);
//
//void MSG_Response_NOK(char* Buffer, UART_HandleTypeDef Uart, int Command);
//
//void MSG_SendVersion(char* Buffer, UART_HandleTypeDef Uart);

#endif
