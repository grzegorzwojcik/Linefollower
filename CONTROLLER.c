/*
 * CONTROLLER.c
 *
 *  Created on: Mar 13, 2015
 *      Author: Grzegorz
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "CONTROLLER.h"


void PID_initSTRUCTURE(void){
	PID_Struct.Threshold = 5000;
	PID_Struct.BaseSpeed = 0;

	PID_Struct.Kp = 1;
	PID_Struct.Ki = 0;
	PID_Struct.Kd = 0;

	PID_Struct.Error_setpoint	 = 0;
	PID_Struct.Error_current	 = 0;
}

int16_t PID_controller(void){
	static uint8_t s = 0;
	static int16_t output = 0;
	static int16_t error_previous = 0;
	static int16_t error_derivative = 0;
	static float error_integral = 0;

	/* Derivative and Integral term updated every 10 times slower than P */
	if( s >= 5 ){
		//error_integral += (PID_Struct.Ki/1000)* PID_Struct.Error_current;
		error_derivative = PID_Struct.Kd * (PID_Struct.Error_current - error_previous);
		s = 0;
		error_previous = PID_Struct.Error_current;	/* Zapamietanie bledu */
		/* Anti wind-up	*/
		//if( error_integral > 50 )
			//error_integral = 50;
		//if( error_integral < -50 )
			//error_integral = -50;
	}
	output = PID_Struct.Kp * PID_Struct.Error_current
					+ error_integral
					+ error_derivative;
	s++;
	return output;
}

uint16_t SENSOR_Calibration(void){
	static uint16_t threshold = 0;
	threshold = ((buforADC[6]+buforADC[12])/2)-200;
	if(threshold > 5000 )
		threshold = 5000;

	return threshold;
}

int16_t SENSOR_ProcessData(uint16_t threshold){

	static uint16_t sensor_array[13] = {0};		//array storing current sensor values and converted later into {0,1} array
	static int16_t sensor_weight[13] = {-35, -25, -20, -15, -10, -5,
										 0,
										 5, 10, 15, 20, 25, 35};

	static uint8_t sensor_count = 0;	// sensor currently above black line counter
	static int16_t weight_sum	= 0;
	static int16_t error = 0;
	static int16_t error_tmp = 0;

	/*		Comparing obtained sensor data (sensor_array) with (threshold),
	 * 		counting sensors above black line (sensor_count)
	 * 		as a result sensor_array becomes {0,1} table	*/
	static uint8_t i = 0;
	for(i = 1, sensor_count = 0; i < 12; i++)
	{
		if(buforADC[i] > threshold){
			sensor_array[i] = 1;
			sensor_count++;
		}
		else
			sensor_array[i] = 0;
	}

	/*		Multiplying {0,1} sensor_array by sensor_weight,
	 * 		summing up total error	(weight_sum)			*/
	for( i = 1, weight_sum = 0; i < 12; i++){
		if( sensor_array[i] > 0 ){
			sensor_array[i] = sensor_array[i]*sensor_weight[i];
			weight_sum += sensor_array[i];
		}
	}

	if( sensor_count > 0){
		error = weight_sum/sensor_count;	// counting error which means how far from the middle sensor the black line is
											// when sensor_count == 0, the program remembers previous error
		error_tmp = error;
	}

	if( sensor_count == 0 )
		error = 2*error_tmp;					// double the error in case the lane was lost

	return error;
}
