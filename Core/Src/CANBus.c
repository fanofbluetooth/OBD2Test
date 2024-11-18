
#include <stdio.h>
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "CANBus.h"
#include <stdbool.h>





#define CAN_RETRANS_TIMES	10
#define CAN_RX_TIME_OUT		3000

CAN_FilterTypeDef			sFilterConfig;
CAN_TxHeaderTypeDef   		TxHeader;
CAN_RxHeaderTypeDef   		RxHeader;
uint32_t					TxMailBox;
uint8_t 					RxData[256];
volatile bool				isCANRXReady;

PROTOCOL_TYPES_t 			proType = PROTO_CAN_NONE;

uint8_t TxData[8];

extern CAN_HandleTypeDef hcan;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	memset(&RxHeader, 0, sizeof(RxHeader));
	memset(RxData, 0, sizeof(RxData));
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	isCANRXReady = true;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	memset(&RxHeader, 0, sizeof(RxHeader));
	memset(RxData, 0, sizeof(RxData));
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	isCANRXReady = true;
}

void CANBusInit(void)
{
	memset(&RxHeader, 0, sizeof(RxHeader));
	memset(RxData, 0, sizeof(RxData));
}

bool CANBusSendData(bool is11Bit, uint8_t* pData, uint8_t len)
{
	static uint32_t tick = 0;
	if((HAL_GetTick() - tick) < 60) HAL_Delay(60);
	tick = HAL_GetTick();
	uint8_t buf[8] = {0x12};
	//if(is11Bit)
	{
		TxHeader.StdId = 0;
		TxHeader.IDE= CAN_ID_STD;
	}		
	TxHeader.RTR= CAN_RTR_DATA;
	TxHeader.DLC= len;
	TxHeader.ExtId = 0x18EF2100;
	TxHeader.TransmitGlobalTime = DISABLE;

	memset(RxData, 0, sizeof(RxData));	
	if( HAL_CAN_GetTxMailboxesFreeLevel( &hcan ) == 0 ) {

		HAL_CAN_AbortTxRequest( &hcan, TxMailBox );
	}
	/* Request transmission */
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &TxMailBox) != HAL_OK)
	{
		return false;
	}
  
	/* Wait transmission complete */
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}
	return true;	
}

bool CANBusGetData(void)
{
	for(uint8_t i = 0; i < 30; i++)
	{
		if(isCANRXReady) return true;
		HAL_Delay(100);
	}
	isCANRXReady = false;
	return false;	
}


 bool CANBusCheckPID(bool is11Bit)
{
	uint8_t pData[8] = {2, 1, 0, 0, 0, 0, 0, 0};
	for(uint8_t i = 0; i < CAN_RETRANS_TIMES; i ++)
	{
		if(i == (CAN_RETRANS_TIMES - 1)) return false;
		if(CANBusSendData(is11Bit, pData, 8)) break;		
	}
	if(RxData[1] != 0x41) return false;
	if(RxData[2] != 0) 	  return false;

	
	return true;

}
bool CANBusInit_11B_500K(void)
{
/*	HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeInit(&hcan);
	  hcan.Instance = CAN1;
	  hcan.Init.Prescaler = 8;
	  hcan.Init.Mode = CAN_MODE_NORMAL;
	  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
	  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	  hcan.Init.TimeTriggeredMode = DISABLE;
	  hcan.Init.AutoBusOff = DISABLE;
	  hcan.Init.AutoWakeUp = DISABLE;
	  hcan.Init.AutoRetransmission = DISABLE;
	  hcan.Init.ReceiveFifoLocked = DISABLE;
	  hcan.Init.TransmitFifoPriority = DISABLE;


	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}	
	*/
	sFilterConfig.FilterMode= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh= 0;// 0x7E8 << 5;
	sFilterConfig.FilterIdLow= 0;
	sFilterConfig.FilterMaskIdHigh= 0;//0x7E8 << 5;
	sFilterConfig.FilterMaskIdLow= 0;
	sFilterConfig.FilterFIFOAssignment= CAN_RX_FIFO1;
	sFilterConfig.FilterActivation= CAN_FILTER_ENABLE;
	sFilterConfig.FilterBank= 10;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_Delay(1000);
	if(CANBusCheckPID(true)) 
	{
		proType = PROTO_CAN_11B_500K;
		return true;
	}
	proType = PROTO_CAN_NONE;
	return false;
}

bool CANBusInit_11B_250K(void)
{
	HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeInit(&hcan);
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}	
	sFilterConfig.FilterMode= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale= CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh= 0x7E8 << 5;
	sFilterConfig.FilterIdLow= 0;
	sFilterConfig.FilterMaskIdHigh= 0x7E8 << 5;
	sFilterConfig.FilterMaskIdLow= 0;
	sFilterConfig.FilterFIFOAssignment= CAN_RX_FIFO0;
	sFilterConfig.FilterActivation= ENABLE;
	sFilterConfig.FilterBank= 13;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(1000);
	if(CANBusCheckPID(true)) 
	{
		proType = PROTO_CAN_11B_250K;
		return true;
	}
	proType = PROTO_CAN_NONE;
	return false;
}

bool CANBusInit_29B_500K(void)
{
	HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeInit(&hcan);
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}	
	sFilterConfig.FilterMode= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x18daf110 >> 13) & 0xFFFF; // Upper 16 bits
	sFilterConfig.FilterIdLow =  (0x18daf110 << 3) & 0xFFF8;   // Lower 13 bits
	sFilterConfig.FilterMaskIdHigh = (0x1FFFFFFF >> 13) & 0xFFFF; // Mask upper 16 bits
	sFilterConfig.FilterMaskIdLow = (0x1FFFFFFF << 3) & 0xFFF8;   // Mask lower 13 bits
	sFilterConfig.FilterFIFOAssignment= CAN_RX_FIFO0;
	sFilterConfig.FilterActivation= ENABLE;
	sFilterConfig.FilterBank= 13;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(1000);
	if(CANBusCheckPID(false)) 
	{
		proType = PROTO_CAN_29B_500K;
		return true;
	}
	proType = PROTO_CAN_NONE;
	return false;
}

bool CANBusInit_29B_250K(void)
{
	HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeInit(&hcan);
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}	
	sFilterConfig.FilterMode= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x18daf110 >> 13) & 0xFFFF; // Upper 16 bits
	sFilterConfig.FilterIdLow =  (0x18daf110 << 3) & 0xFFF8;   // Lower 13 bits
	sFilterConfig.FilterMaskIdHigh = (0x1FFFFFFF >> 13) & 0xFFFF; // Mask upper 16 bits
	sFilterConfig.FilterMaskIdLow = (0x1FFFFFFF << 3) & 0xFFF8;   // Mask lower 13 bits
	sFilterConfig.FilterFIFOAssignment= CAN_RX_FIFO0;
	sFilterConfig.FilterActivation= ENABLE;
	sFilterConfig.FilterBank= 13;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(1000);
	if(CANBusCheckPID(false)) 
	{
		proType = PROTO_CAN_29B_250K;
		return true;
	}
	proType = PROTO_CAN_NONE;
	return false;
}

bool CANBusInit_ISO_15765_4(void)
{
	if (CANBusInit_11B_500K()) return true;
	if (CANBusInit_11B_250K()) return true;
	if (CANBusInit_29B_500K()) return true;
	if (CANBusInit_29B_250K()) return true;
	return false;
}

bool CANBusInit_AUTO(void)
{
	if (CANBusInit_ISO_15765_4()) return true;
	return false;
}

bool CANSendPathThrough(uint8_t* pData, uint8_t len)
{
	bool ret = false;
	switch(proType)
	{
		case PROTO_CAN_11B_250K:
		case PROTO_CAN_11B_500K:
			if (CANBusSendData(true, pData, len)) ret = true;
		break;
		case PROTO_CAN_29B_250K:
		case PROTO_CAN_29B_500K:
			if (CANBusSendData(false, pData, len)) ret = true;
		break;		
		default:
		break;
	}
	return ret;
}

bool CANRxHandler(uint32_t* pHeader, uint8_t* pData)
{
	if (isCANRXReady)
	{
		isCANRXReady = false;
		memcpy(pHeader, &RxHeader, sizeof(RxHeader));
		memcpy(pData, RxData, 8);
		return true;
	}
	return false;
}


void CANTxTest()
{
	TxData[0] = 0x04;
	TxData[1] = 0x1b;
	TxData[2] = 0x7D;
	TxData[3] = 0x00;
	TxData[4] = 0xff;
	TxData[5] = 0xff;
	TxData[6] = 0xff;
	TxData[7] = 0xff;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}
