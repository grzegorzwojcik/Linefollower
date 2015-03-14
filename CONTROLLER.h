/*
 * CONTROLLER.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Grzegorz
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"

extern volatile uint16_t buforADC[14];

typedef struct{

	uint16_t	Threshold;
	uint16_t	BaseSpeed;

	uint8_t	 Kp;
	float	 Ki;
	uint8_t	 Kd;

	int16_t Error_setpoint;
	int16_t Error_current;
	int16_t Error_previous;
	int16_t Error_sum;

}PID, *pPID;

volatile PID PID_Struct;

void PID_initSTRUCTURE(void);
uint16_t SENSOR_Calibration(void);
int16_t SENSOR_ProcessData(uint16_t threshold);

#endif /* CONTROLLER_H_ */
