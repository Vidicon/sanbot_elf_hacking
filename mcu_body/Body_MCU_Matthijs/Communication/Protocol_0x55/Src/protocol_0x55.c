#include "protocol_0x55.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "DistanceSensors.h"
#include "Compass.h"
#include "Battery.h"

static struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_RxData;
static struct PROTOCOL_0X55_Data_Type PROTOCOL_0X55_TxData;

// volatile to indicate this variable can be changed at any time.
static volatile uint8_t RxMutex;
static UART_HandleTypeDef *Uart_0x55;

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

void Protocol_0x55_Init(UART_HandleTypeDef *huart)
{
	PROTOCOL_0X55_RxData.BytesInBuffer = 0;
	PROTOCOL_0X55_TxData.BytesInBuffer = 0;

	// Clear the buffers
	memset(&PROTOCOL_0X55_RxData.FIFO_Data[0], 0, FIFO_SIZE);
	memset(&PROTOCOL_0X55_TxData.FIFO_Data[0], 0, FIFO_SIZE);

	Uart_0x55 = huart;
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

	sprintf(&Buffer[3], "SANBOT-BODY by MatthijsFH - TAG V2.0 - ");
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
	// USB DMA tx
	//	CDC_Transmit_FS((uint8_t*)data, 3 + payloadLen + 2);

	HAL_UART_Transmit_DMA(Uart_0x55, (uint8_t *)data, 3 + payloadLen + 2);
}

signed char Protocol_0x55_GetData(int Index)
{
	return PROTOCOL_0X55_RxData.FIFO_Data[Index];
}


void SendEncoders(struct Encoders_Data_Type *EncoderData)
{
	Protocol_0x55_SendEncoders((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], EncoderData);
}


void Protocol_0x55_SendEncoders(char *Buffer, struct Encoders_Data_Type *EncoderData)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_GET_ENCODERS, RESPONSE_TRUE);

	Buffer[3] = (EncoderData->Encoder[0] >> 8) & 0xff;
	Buffer[4] = (EncoderData->Encoder[0]) & 0xff;

	Buffer[5] = (EncoderData->Encoder[1] >> 8) & 0xff;
	Buffer[6] = (EncoderData->Encoder[1]) & 0xff;

	Buffer[7] = (EncoderData->Encoder[2] >> 8) & 0xff;
	Buffer[8] = (EncoderData->Encoder[2]) & 0xff;

	Buffer[9] = (EncoderData->Encoder[3] >> 8) & 0xff;
	Buffer[10] = (EncoderData->Encoder[3]) & 0xff;

	Buffer[11] = (EncoderData->Encoder[4] >> 8) & 0xff;
	Buffer[12] = (EncoderData->Encoder[4]) & 0xff;

	int payloadLen = 10;

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

//----------------------------------------------------------------
// Motion sensors
//----------------------------------------------------------------
void SendMotionSensors(struct MotionSensors_Data_Type *MotionSensors_State)
{
	Protocol_0x55_SendMotionEvent((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], MotionSensors_State);
}


void Protocol_0x55_SendMotionEvent(char *Buffer, struct MotionSensors_Data_Type *MotionSensors_State)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_GET_MOTIONSENSORS, RESPONSE_TRUE);

	Buffer[3] = MotionSensors_State->CurrentValue[0];
	Buffer[4] = MotionSensors_State->CurrentValue[1];

	int payloadLen = 2;

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

//----------------------------------------------------------------
// Distance sensors
//----------------------------------------------------------------
void SendDistanceSensors(struct Distance_Sensor_Type *DistanceData)
{
	Protocol_0x55_SendDistanceEvent((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], DistanceData);
}

void Protocol_0x55_SendDistanceEvent(char *Buffer, struct Distance_Sensor_Type *DistanceData)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_GET_DISTANCESENSORS, RESPONSE_TRUE);

	for (int i = 0; i < NO_DISTANCE_SENSORS; i++)
	{
		Buffer[3 + i*2] = (DistanceData->Distance[i] >> 8);
		Buffer[4 + i*2] = (DistanceData->Distance[i] & 0xff);
	}

	int payloadLen = NO_DISTANCE_SENSORS * 2;

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

//----------------------------------------------------------------
// Raw compass data
//----------------------------------------------------------------
void SendCompass(struct Compass_Sensor_Type *CompassData)
{
	Protocol_0x55_SendCompass((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], CompassData);
}

void Protocol_0x55_SendCompass(char *Buffer, struct Compass_Sensor_Type *CompassData)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_GET_COMPASS, RESPONSE_TRUE);

	int payloadLen = 6;

	Buffer[3 + 0] = (CompassData->Raw[1]);
	Buffer[3 + 1] = (CompassData->Raw[0]);

	Buffer[3 + 2] = (CompassData->Raw[3]);
	Buffer[3 + 3] = (CompassData->Raw[2]);

	Buffer[3 + 4] = (CompassData->Raw[5]);
	Buffer[3 + 5] = (CompassData->Raw[4]);

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

//----------------------------------------------------------------
// Battery data
//----------------------------------------------------------------
void SendBattery(struct Battery_Sensor_Type *BatteryData)
{
	Protocol_0x55_SendBattery((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], BatteryData);
}

void Protocol_0x55_SendBattery(char *Buffer, struct Battery_Sensor_Type *BatteryData)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_GET_BATTERY, RESPONSE_TRUE);

	int payloadLen = 14;

	// 	uint16_t DeviceType;
	//	uint16_t FW_Version;
	//	uint16_t HW_Version;
	//	uint16_t Temperature;
	//	int16_t Current;
	//	uint16_t Voltage;
	//	uint16_t BatteryState;

	Buffer[3 + 0] = (BatteryData->DeviceType >> 8 );
	Buffer[3 + 1] = (BatteryData->DeviceType & 0xff);

	Buffer[3 + 2] = (BatteryData->FW_Version >> 8 );
	Buffer[3 + 3] = (BatteryData->FW_Version & 0xff);

	Buffer[3 + 4] = (BatteryData->HW_Version >> 8 );
	Buffer[3 + 5] = (BatteryData->HW_Version & 0xff);

	Buffer[3 + 6] = (BatteryData->BatteryState >> 8 );
	Buffer[3 + 7] = (BatteryData->BatteryState & 0xff);

	Buffer[3 + 8] = (BatteryData->Temperature >> 8 );
	Buffer[3 + 9] = (BatteryData->Temperature & 0xff);

	Buffer[3 + 10] = (BatteryData->Current >> 8 );
	Buffer[3 + 11] = (BatteryData->Current & 0xff);

	Buffer[3 + 12] = (BatteryData->Voltage >> 8 );
	Buffer[3 + 13] = (BatteryData->Voltage & 0xff);


	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}

//----------------------------------------------------------------
// Raw compass data
//----------------------------------------------------------------
void SendCompassMoveDone(uint8_t Succes)
{
	Protocol_0x55_SendCompassMoveDone((char *) &PROTOCOL_0X55_TxData.FIFO_Data[0], Succes);
}

void Protocol_0x55_SendCompassMoveDone(char *Buffer, uint8_t Succes)
{
	Protocol_0x55_PrepareNewMessage(Buffer, CMD_COMP_MOVE, RESPONSE_TRUE);

	int payloadLen = 1;

	Buffer[3 + 0] = Succes;

	Protocol_0x55_SetLength(Buffer, payloadLen);
	Protocol_0x55_AddCRC(Buffer, payloadLen);
	Protocol_0x55_Send(Buffer, payloadLen);
}
