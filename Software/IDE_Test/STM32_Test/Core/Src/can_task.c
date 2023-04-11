/*
 * can_task.c
 *
 *  Created on: Mar 9, 2023
 *      Author: kate
 */
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

typedef enum {
	temperatureAddress = 0x0A2,
	txAddress = 0x0C0,
} motorAddress;

CAN_TxHeaderTypeDef   txCAN;
CAN_RxHeaderTypeDef  RxHeader;
uint32_t txMail;
uint8_t RxData[8], txData[8];

//int temp
int motorTorqueHighFault = 0x00, motorTorqueLowFault = 0x00, decodedTemperature;

//grab the hex temperature codes from the CAN frame and convert it into decimal values
int temperatureDecode (int high, int low){
	 int temperatureCelsius = (high*256 + low)/10;
	return temperatureCelsius;
}
//initialize CAN filters for TX/RX operations
void can_Init(void){
	CAN_FilterTypeDef  sFilterConfig;
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
	//throw error if configurations are unsuccessful
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK){
	  /* Filter configuration Error */
	  Error_Handler();
	}
	// start the CAN perpherial via HAL
	HAL_CAN_Start(&hcan1);
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
	/* Notification Error */
	Error_Handler();
  }
}
//send can frame function
void canSend(void){
	txCAN.ExtId = txAddress;
	txCAN.DLC = 2;
	txData[0] = motorTorqueLowFault;
	txData[1] = motorTorqueHighFault;
	HAL_CAN_AddTxMessage(&hcan1, &txCAN, txData, &txMail);
}
//decode the message from the interrupt with queueget
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
	/* Reception Error */
	Error_Handler();
  }
  //decode the specified hex bytes that contain motor temperature
  if (RxHeader.ExtId == temperatureAddress){
	  decodedTemperature = temperatureDecode(RxData[5], RxData[4]);
  }
}
