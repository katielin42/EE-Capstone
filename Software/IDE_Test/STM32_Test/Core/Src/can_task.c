/*
 * can_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#include "main.h"
#include "cmsis_os.h"

typedef enum {
	temperatureAddress = 0x0A2,
	txAddress = 0x0C0,
} motorAddress;

//CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef   txCAN;
CAN_RxHeaderTypeDef  RxHeader;
uint32_t txMail;
uint8_t RxData[8];
uint8_t txData[8];

//osMessageQueueId_t canmsg_rx;
//
//osMessageQueueAttr_t msgAttr = {
//		.name = "can_rx",
//};
//
//typedef struct  {
//	CAN_RxHeaderTypeDef  RxHeader;
//	uint8_t RxData[8];
//}canMsg;




//int temp;
int motorTorqueHighFault = 0x00;
int motorTorqueLowFault = 0x00;
int decodedTemperature;

int temperatureDecode (int high, int low) {
	 int temperatureCelsius = (high*256 + low)/10;
	return temperatureCelsius;
}
unsigned int revHex(unsigned int hex_num) {
    unsigned int reversed_num = 0;
    while (hex_num > 0) {
        reversed_num = (reversed_num << 4) + (hex_num & 0xF);
        hex_num >>= 4;
    }
    return reversed_num;
}
void can_Init(void) {
	CAN_FilterTypeDef  sFilterConfig;
	//canmsg_rx = osMessageQueueNew(10, sizeof(canMsg), &msgAttr);
	txCAN.IDE = CAN_ID_EXT;
	txCAN.RTR = CAN_RTR_DATA;
	txCAN.TransmitGlobalTime = DISABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
	    /* Filter configuration Error */
	  Error_Handler();
	}
	HAL_CAN_Start(&hcan1);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	/* Notification Error */
	Error_Handler();
  }
}

//thread
void canSend(void) {
	txCAN.ExtId = txAddress;
	txCAN.DLC = 2;
	txData[0] = motorTorqueLowFault;
	txData[1] = motorTorqueHighFault;
//	while(1) {
//		HAL_CAN_AddTxMessage(&hcan1, &txCAN, txData, &txMail);
////		osSemaphoreAcquire()
//		osDelay(50);
//	}
	HAL_CAN_AddTxMessage(&hcan1, &txCAN, txData, &txMail);
}

//decode the message from the interrupt with queueget
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//canMsg temp;
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	/* Reception Error */
	Error_Handler();
  }
//  if (temp.RxHeader.ExtId == temperatureAddress) {
//	  decodedTemperature = temperatureDecode(temp.RxData[1], temp.RxData[0]);
////	  osMessageQueuePut(canmsg_rx, &decodedTemperature, 0, 0);
//  }
  if (RxHeader.ExtId == temperatureAddress) {
	  decodedTemperature = temperatureDecode(RxData[5], RxData[4]);
//	  osMessageQueuePut(canmsg_rx, &decodedTemperature, 0, 0);
  }
//  osMessageQueuePut(canmsg_rx, &temp, 0 ,0);

  //
//  if (RxHeader.ExtId == temperatureAddress) {
//	  decodedTemperature = temperatureDecode(RxData[5], RxData[4]);
//  }
}

//int canRecieve(void) {
//	int decodedTemperature;
//
//	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//	  /* Notification Error */
//	  Error_Handler();
//	}
//
//	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//
//	  /* Get RX message */
//	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//	  {
//		/* Reception Error */
//		Error_Handler();
//	  }
//	  if (RxHeader.ExtId == temperatureAddress) {
//		  decodedTemperature = temperatureDecode(RxData[5], RxData[4]);
//	  }
//	}
//
//	return decodedTemperature;
//}



