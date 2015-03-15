/*
 * ADC.h
 *
 *  Created on: Jun 16, 2014
 *      Author: Grzegorz Wojcik
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

/*	PC0 - Sensor 00 - ADC1 CHANNEL 10 | PC1 - Sensor 01 - ADC1 CHANNEL 11 | PC2 - Sensor 02 - ADC1 CHANNEL 12
	PC3 - Sensor 03 - ADC1 CHANNEL 13 | PC4 - Sensor 12 - ADC1 CHANNEL 14 |

	PA0 - Sensor 04 - ADC1 CHANNEL 00
	PA1 - Sensor 05 - ADC1 CHANNEL 01 | PA2 - Sensor 06 - ADC1 CHANNEL 02 | PA3 - Sensor 07 - ADC1 CHANNEL 03
	PA4 - Sensor 08 - ADC1 CHANNEL 04 | PA5 - Sensor 09 - ADC1 CHANNEL 05 | PA6 - Sensor 10 - ADC1 CHANNEL 06
	PA7 - Sensor 11 - ADC1 CHANNEL 07

	PC5 - 3-PIN JUMPER BETWEEN IR DISTANCE SENSOR AND LI-PO VOLTAGE MEASUREMENT
*/
volatile uint16_t buforADC[14];

/**
  * @brief  ADC1 initialization
  * @param  None
  * @retval None
  */
void ADC_init(void);

/**
  * @brief  ADC1 GPIO initialization
  * @param  None
  * @retval None
  */
void ADC_initGPIO(void);

/**
  * @brief  DMA1 initialization
  * @param  None
  * @retval None
  */
void ADC_initDMA(void);

/**
  * @brief  Li-po voltage measurement (powering up RED LED when low voltage level)
  * @param  None
  * @retval None
  */
void ADC_BatteryMonitor(void);


#endif /* ADC_H_ */
