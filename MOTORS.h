/*
 * MOTORS.h
 *
 *  Created on: Feb 15, 2014
 *      Author: Grzegorz
 */
#include <stddef.h>
#include "stm32f10x.h"

#ifndef MOTORS_H_
#define MOTORS_H_

void MOTORS_initPOWERSUPPLY(void);
void MOTORS_initGPIO(void);
void MOTORS_initPWM(void);
void MOTOR_set(int16_t MOT_LEFT, int16_t MOT_RIGHT);

typedef struct{

		uint16_t PWM_RIGHT;
		uint16_t PWM_LEFT;

		uint16_t PWM_off;
		uint16_t PWM_base;
		uint16_t PWM_max;

}MOTORS, *pMOTORS;

#endif /* MOTORS_H_ */
