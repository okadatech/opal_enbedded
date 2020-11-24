/*
 * managiment.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include "can.h"
#include "main.h"
#include "stm32f3xx_hal.h"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "can_ibis.h"
#include "util.h"



float pitchAngle;
float rollAngle;
float yawAngle;
float pitchAngle_rad;
float rollAngle_rad;
float yawAngle_rad;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
CAN_FilterTypeDef  sFilterConfig;


TIM_Encoder_InitTypeDef sConfig;
TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

#define can_RX_data 8
#define can_TX_data 8


#endif /* MANAGEMENT_H_ */
