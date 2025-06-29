#include "protocol_0x55.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

static struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_RxData;
static struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_TxData;

// volatile to indicate this variable can be changed at any time.
static volatile uint8_t RxMutex;

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

void Protocol_0x55_NewData(uint8_t* Buf, uint32_t *Len)
{
	RxMutex = 1;

	memcpy(&PROTOCOL_0X55_RxData.FIFO_Data[PROTOCOL_0X55_RxData.BytesInBuffer], (char*) Buf, *Len);
	PROTOCOL_0X55_RxData.BytesInBuffer += *Len;

	//--------------------------------------------------------------------------------------------------
	// **IF** the lost bytes bug returns, then the new data from the USB device, should be stored in
	// a temp buffer and only be copied inside the Protocol_0x55_CheckFifo() function.
	// The HAL_DELAY(1) in the main program loop seems to do the trick anyway.
	//--------------------------------------------------------------------------------------------------
}

void Protocol_0x55_Init()
{
	PROTOCOL_0X55_RxData.BytesInBuffer = 0;
	PROTOCOL_0X55_TxData.BytesInBuffer = 0;

	// Clear the buffers
	memset(&PROTOCOL_0X55_RxData.FIFO_Data[0], 0, FIFO_SIZE);
	memset(&PROTOCOL_0X55_TxData.FIFO_Data[0], 0, FIFO_SIZE);
}

uint8_t Protocol_0x55_CheckFifo()
{
	// Prevent both Protocol_0x55_NewData() and Protocol_0x55_CheckFifo()
	// from modifying the same data at the same time.
	//  This seems to lead to lost bytes.
	if (RxMutex >= 1)
	{
		RxMutex = 0;
		return 0;
	}

	// No data in buffer
	if (PROTOCOL_0X55_RxData.BytesInBuffer == 0)
	{
		PROTOCOL_0X55_RxData.TotalMsgSize = 0;
		PROTOCOL_0X55_RxData.FIFO_Data[0] = 0;
		return 0;
	}

	//=================================================================================================
	// When the USB connects the 1st time, there seems to be a bug which makes the last TX message appear (partly) in the RX queue
	// Always with the 0xd5 start byte. So easy to detect as there is no other data in the RX queue.
	//=================================================================================================
	if (PROTOCOL_0X55_RxData.FIFO_Data[0] == (0x55 | RESP_BIT))
	{
		PROTOCOL_0X55_RxData.TotalMsgSize = 0;
		PROTOCOL_0X55_RxData.FIFO_Data[0] = 0;
		memset(&PROTOCOL_0X55_RxData.FIFO_Data[0], 0, FIFO_SIZE);

		return 0;
	}

	// Start byte should be 0x55
	// If not, shift whole buffer 1 position.
	if (PROTOCOL_0X55_RxData.FIFO_Data[0] != 0x55)
	{
		// Shift whole buffer 1 byte.
		memmove(&PROTOCOL_0X55_RxData.FIFO_Data[0], &PROTOCOL_0X55_RxData.FIFO_Data[1], FIFO_SIZE-1);

		// Clear last byte
		PROTOCOL_0X55_RxData.FIFO_Data[FIFO_SIZE-1] = 0;

		// Update. 1 bytes less in the buffer
		PROTOCOL_0X55_RxData.BytesInBuffer -= 1;
		PROTOCOL_0X55_RxData.TotalMsgSize = 0;

		return 0;
	}

	// When we are here, the current start byte = 0x55
	// Data length according the received message
	uint8_t datalen = PROTOCOL_0X55_RxData.FIFO_Data[2];

	// Not all data received yet
	if (PROTOCOL_0X55_RxData.BytesInBuffer < (3 + datalen + 2))
	{
		PROTOCOL_0X55_RxData.TotalMsgSize = 0;
		return 0;
	}

	// Enough data received to do a CRC check
	uint16_t Result = Protocol_0x55_CalculateCRC16((char*)&PROTOCOL_0X55_RxData.FIFO_Data[0], 3+datalen+2);

	if ((Result & 0xff) == PROTOCOL_0X55_RxData.FIFO_Data[datalen+3])
	{
		PROTOCOL_0X55_RxData.TotalMsgSize = 3 + datalen + 2;

		return 1;
	}
	else
	{
		// Start = 0x55, enough data but CRC not ok.
		// Clean the 0x55, so receiver starts to shift.
		PROTOCOL_0X55_RxData.FIFO_Data[0] = 0;
		PROTOCOL_0X55_RxData.TotalMsgSize = 0;

		return 0;
	}
}

void Protocol_0x55_MarkProcessed()
{
//	// Clear first byte. Not very nice but works.
//	// Receiver will start shifting until new 0x55 is found or BytesInBuffer = 0
//	PROTOCOL_0X55_RxData.FIFO_Data[0] = 0;

	int MsgSize = PROTOCOL_0X55_RxData.TotalMsgSize;

	// Shift whole buffer
	memmove(&PROTOCOL_0X55_RxData.FIFO_Data[0], &PROTOCOL_0X55_RxData.FIFO_Data[MsgSize], FIFO_SIZE - MsgSize);

	// Clear last byte
	PROTOCOL_0X55_RxData.FIFO_Data[FIFO_SIZE-1] = 0;

	// Update. 1 bytes less in the buffer
	PROTOCOL_0X55_RxData.BytesInBuffer -= MsgSize;
}

int Protocol_0x55_GetCommand()
{
	return PROTOCOL_0X55_RxData.FIFO_Data[1];
}

void Protocol_0x55_ClearRxBuffer()
{
	memset((uint8_t*)PROTOCOL_0X55_RxData.FIFO_Data, 0, sizeof(PROTOCOL_0X55_RxData.FIFO_Data));
}

void SendVersion(void)
{
	Protocol_0x55_SendVersion((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0]);
}

void Protocol_0x55_SendVersion(char *Buffer)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_VERSION, RESPONSE_TRUE);

	sprintf(&Buffer[3], "SANBOT-HEAD by MatthijsFH - TAG V2.0 - ");
	sprintf(&Buffer[3 + strlen(&Buffer[3])], __TIME__);
	sprintf(&Buffer[3 + strlen(&Buffer[3])], " ");
	sprintf(&Buffer[3 + strlen(&Buffer[3])], __DATE__);

	int payloadLen = strlen(&Buffer[3]);

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

void Protocol_0x55_PrepareNewMessage(char *Buffer, char Command, char Response)
{
	memset((uint8_t*)Buffer, 0, FIFO_SIZE);

	// Fix for nasty bug
	Buffer[0] = 0x55 | 0x80;
	Buffer[1] = (Command & 0x7f);

	if (Response == 1) {Buffer[1] = Buffer[1] | 0x80;}		// Set high bit
}

void Protocol_0x55_SetLength(char *Buffer, uint8_t datalen)
{
	Buffer[2] = datalen;
}

void Protocol_0x55_AddCRC(char *Buffer, uint8_t payloadLen)
{
	// 0x55 CMD LEN + payload + CRC1 + CRC2
	uint16_t Result = Protocol_0x55_CalculateCRC16(Buffer, 3 + payloadLen + 2);

	Buffer[3 + payloadLen + 0] 	= (Result & 0xff);
	Buffer[3 + payloadLen + 1]	= ((Result >> 8) & 0xff);
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

void Protocol_0x55_Send(char *data, uint8_t payloadLen)
{
	CDC_Transmit_FS((uint8_t*)data, 3 + payloadLen + 2);
}

signed char Protocol_0x55_GetData(int Index)
{
	return PROTOCOL_0X55_RxData.FIFO_Data[Index];
}

//----------------------------------------------------------------
// Touch sensors
//----------------------------------------------------------------
void SendTouchSensors(struct TouchSensors_Data_Type *TouchData)
{
	Protocol_0x55_SendTouchEvent((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], TouchData);
}

void Protocol_0x55_SendTouchEvent(char *Buffer, struct TouchSensors_Data_Type *TouchData)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_HEAD_TOUCHSENSORS, RESPONSE_TRUE);

	for (int i = 0; i < NO_TOUCH_SENSORS; i++)
	{
		Buffer[3 + i] = (TouchData->Sensor[i]);
	}

	int payloadLen = NO_TOUCH_SENSORS;

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

