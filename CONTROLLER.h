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
}PID, *pPID;

volatile PID PID_Struct;

void PID_initSTRUCTURE(void);
/**
  * @brief  PID control loop
  * @param  None
  * @retval None
  */
int16_t PID_controller(void);

/**
  * @brief  SENSOR calibration function. Black line should be placed under
  * 			sensor 13, and white surface under sensor 7 (middle one)
  * @param  None
  * @retval None
  */
uint16_t SENSOR_Calibration(void);

/**
  * @brief  Function calculating error, which means how far, from the middle sensor, the black line is
  * @param  threshold calculated by SENSOR_calibration (and volatile sensor table ADC_bufor[13])
  * @retval error
  */
int16_t SENSOR_ProcessData(uint16_t threshold);

#endif /* CONTROLLER_H_ */
