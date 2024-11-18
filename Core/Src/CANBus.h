#ifndef CAN_BUS_H_
#define CAN_BUS_H_
#include "main.h"
#include <stdbool.h>

typedef enum
{
	PROTO_AUTO = '0',
	PROTO_ISO_15765_4,
	PROTO_ISO_9141_2,
	PROTO_KWP2000_5KBPS,
	PROTO_KWP2000_FAST,
	PROTO_CAN_11B_500K,
	PROTO_CAN_29B_500K,
	PROTO_CAN_29B_250K,
	PROTO_CAN_11B_250K,
	PROTO_CAN_NONE
} PROTOCOL_TYPES_t;


void CANBusInit(void);
bool CANBusCheckPID(bool is11Bit);
bool CANBusInit_AUTO(void);
bool CANBusInit_ISO_15765_4(void);
bool CANBusInit_11B_500K(void);
bool CANBusInit_11B_250K(void);
bool CANBusInit_29B_500K(void);
bool CANBusInit_29B_250K(void);
bool CANSendPathThrough(uint8_t* pData, uint8_t len);
bool CANRxHandler(uint32_t* pHeader, uint8_t* pData);

#endif

