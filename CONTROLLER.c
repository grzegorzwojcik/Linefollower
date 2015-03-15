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
	PID_Struct.Threshold = 7000;
	PID_Struct.BaseSpeed = 0;

	PID_Struct.Kp = 1;
	PID_Struct.Ki = 0;
	PID_Struct.Kd = 1;

	PID_Struct.Error_setpoint	 = 0;
	PID_Struct.Error_current	 = 0;
	PID_Struct.Error_previous	 = 0;
	PID_Struct.Error_sum		 = 0;

}


/**
  * @brief  SENSOR calibration function. Black line should be placed under
  * 			sensor 13, and white surface under sensor 7 (middle one)
  * @param  None
  * @retval None
  */
uint16_t SENSOR_Calibration(void){
	static uint16_t threshold = 0;
	threshold = ((buforADC[6]+buforADC[12])/2)-200;
	if(threshold > 5000 )
		threshold = 5000;

	return threshold;
}

/**
  * @brief  Function calculating error, which means how far, from the middle sensor, the black line is
  * @param  threshold calculated by SENSOR_calibration (and volatile sensor table ADC_bufor[13])
  * @retval error
  */
int16_t SENSOR_ProcessData(uint16_t threshold){

	static uint16_t sensor_array[13] = {0};
	static int16_t sensor_weight[13] = {-35, -25, -20, -15, -10, -5,
										 0,
										 5, 10, 15, 20, 25, 35};

	static uint8_t sensor_count = 0;	// sensor currently above black line counter
	static int16_t weight_sum	= 0;
	static int16_t error = 0;

	/*		Comparing obtained sensor data (sensor_array) with (threshold),
	 * 		counting sensors above black line (sensor_count)
	 * 		as a result sensor_array becomes {0,1} table	*/
	static uint8_t i = 0;
	for(i = 0, sensor_count = 0; i < 13; i++)
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
	for( i = 0, weight_sum = 0; i < 13; i++){
		if( sensor_array[i] > 0 ){
			sensor_array[i] = sensor_array[i]*sensor_weight[i];
			weight_sum += sensor_array[i];
		}
	}

	if( sensor_count > 0)
		error = weight_sum/sensor_count;	// counting error which means how far from the middle sensor the black line is
											// when sensor_count == 0, the program remembers previous error
	return error;
}
