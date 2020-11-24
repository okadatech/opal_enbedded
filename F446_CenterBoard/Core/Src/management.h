/*
 * managiment.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#include "main.h"
#include "can.h"
#include "dac.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "can_ibis.h"
#include "util.h"
#include "DFPlayer_Mini_mp3.h"
#include "microsectimer.h"
#include "mpu9250.h"
#include "myatan2.h"

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


TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
#define  Rxbufsize_from_ESP32 5
uint8_t Rxbuf_from_ESP32[Rxbufsize_from_ESP32];
uint8_t data_from_esp32[Rxbufsize_from_ESP32-1];
uint8_t Rxbuf_from_ESP32_temp[Rxbufsize_from_ESP32-1];


#define can_RX_data 8
#define can_TX_data 8


#endif /* MANAGEMENT_H_ */
