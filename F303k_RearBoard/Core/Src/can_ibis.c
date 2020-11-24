/*
 * can_ibis.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#include "can_ibis.h"




void can_init(CAN_HandleTypeDef* handler){
	  CAN_ibis = handler;
	  CAN_FilterTypeDef  sFilterConfig;
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode =  CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	  sFilterConfig.FilterIdHigh = 0x000;
	  sFilterConfig.FilterIdLow = 0x000;
	  sFilterConfig.FilterMaskIdHigh = 0x000;
	  sFilterConfig.FilterMaskIdLow = 0x000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;

	  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK){  Error_Handler();}
	  if (HAL_CAN_Start(&hcan) != HAL_OK){ Error_Handler();}
	  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {Error_Handler(); }
}

void can_send(int id, uint8_t senddata[8]){

	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	/* Request transmission */
	if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,senddata, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);

}

