/*
 * can_ibis.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef CAN_IBIS_H_
#define CAN_IBIS_H_

#include <stdbool.h>
#include <string.h>
#include "management.h"

CAN_HandleTypeDef* CAN_ibis;
void can_init(CAN_HandleTypeDef* handler);
void can_send(int id, uint8_t senddata[]);


#endif /* CAN_IBIS_H_ */
