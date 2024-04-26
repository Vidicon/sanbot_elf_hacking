#include "protocol_0x55.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_RxData;
struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_TxData;

// Allow other modules to retreive the Rx pointer
struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetRxPointer()
{
	return (&PROTOCOL_0X55_RxData);
}

// Allow other modules to retreive the Tx pointer
struct PROTOCOL_0X55_Data_Type* Protocol_0x55_GetTxPointer()
{
	return (&PROTOCOL_0X55_TxData);
}

void Protocol_0x55_Init()
{
	PROTOCOL_0X55_RxData.NewData = 0;
	PROTOCOL_0X55_TxData.NewData = 0;
}

uint8_t Protocol_0x55_CheckFifo()
{
	return PROTOCOL_0X55_RxData.NewData;
}

void Protocol_0x55_ProcessRxCommand()
{
	PROTOCOL_0X55_RxData.NewData = 0;

	Protocol_0x55_SendVersion((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0]);
}

void Protocol_0x55_SendVersion(char *Buffer)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_VERSION, RESPONSE_TRUE);

	sprintf(&Buffer[3], "STM32F207 - Sanbot - Matthijs ");
	sprintf(&Buffer[3 + strlen(&Buffer[3])], __TIME__);
	sprintf(&Buffer[3 + strlen(&Buffer[3])], " ");
	sprintf(&Buffer[3 + strlen(&Buffer[3])], __DATE__);

	int datalen = strlen(&Buffer[3]);

	Protocol_0x55_SetLength(Buffer, datalen);
	Protocol_0x55_AddCRC(Buffer, datalen);
	Protocol_0x55_Send(Buffer, datalen);
}

void Protocol_0x55_PrepareNewMessage(char *Buffer, char Command, char Response)
{
	memset((uint8_t*)Buffer, 0, sizeof(Buffer));

	Buffer[0] = 0x55;
	Buffer[1] = (Command & 0x7f);

	if (Response == 1) {Buffer[1] = Buffer[1] | 0x80;}		// Set high bit
}

void Protocol_0x55_SetLength(char *Buffer, uint8_t datalen)
{
	Buffer[2] = datalen;
}

void Protocol_0x55_AddCRC(char *Buffer, uint8_t datalen)
{
	// 0x55 CMD LEN + datalen
	uint16_t Result = Protocol_0x55_CalculateCRC16(Buffer, datalen);

	Buffer[datalen+3] 	= (Result & 0xff);
	Buffer[datalen+4]	= ((Result >> 8) & 0xff);
}

//------------------------------------------------------------------------
// CRC-16 / Modbus version. start at 0xFFFF, 0x8005 reversed
// Reversed because CRC is shifted right instead of left.
//------------------------------------------------------------------------
uint16_t Protocol_0x55_CalculateCRC16(char *data, uint8_t msgSize)
{
	uint16_t crc = 0xFFFF; // Initial value of CRC
	uint16_t crclen = msgSize - 2;	// Substract 2 CRC bytes

	for (uint8_t i = 0; i < crclen; i++) {
		crc ^= data[i]; // XOR the next data byte

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x0001) {
				crc >>= 1;
				crc ^= 0xA001; // Polynomial for CRC-16/MODBUS (0x8005 reversed)
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

void Protocol_0x55_Send(char *data, uint8_t datalen)
{
	CDC_Transmit_FS((uint8_t*)data, 3 + datalen + 2);
}

//void MSG_Response_OK(char* Buffer, UART_HandleTypeDef Uart, int Command)
//{
//	MSG_PrepareNewMessage(Buffer, Command, RESPONSE_TRUE);
//	sprintf(&Buffer[3], "OK");
//	int datalen = strlen(&Buffer[3]);
//
//	MSG_SetLength(Buffer, datalen);
//	MSG_AddCRC(Buffer, datalen);
//	MSG_Send(Buffer, Uart, datalen);
//}
//
//void MSG_Response_NOK(char* Buffer, UART_HandleTypeDef Uart, int Command)
//{
//	MSG_PrepareNewMessage(Buffer, Command, RESPONSE_TRUE);
//	sprintf(&Buffer[3], "NOK");
//	int datalen = strlen(&Buffer[3]);
//
//	MSG_SetLength(Buffer, datalen);
//	MSG_AddCRC(Buffer, datalen);
//	MSG_Send(Buffer, Uart, datalen);
//}


