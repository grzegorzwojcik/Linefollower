/*
 * MOTORS.h
 *
 *  Created on: Feb 15, 2014
 *      Author: Grzegorz
 */

#ifndef MOTORS_H_
#define MOTORS_H_

void MOTORS_init(void);
void PWM_init(void);
void MOTOR_set(int16_t MOT_LEFT, int16_t MOT_RIGHT);


#endif /* MOTORS_H_ */
