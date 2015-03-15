/**
*****************************************************************************
**
**  File        : main.c
**  Abstract    : main function
**	Author		: Grzegorz Wojcik
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include <stdlib.h>

#include "ADC.h"
#include "BTM.h"
#include "functions.h"
#include "MOTORS.h"
#include "stm32f10x.h"

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	a = 0;	// Ustawianie wartosci poczatkowej zmiennych ulotnych

	/* Clearing data frame */
	static uint8_t i = 0;
	for( i = 0; i < 30; i++ ){
	  received_frame[i] = 0;
	}

	/* \/\/\/ SYSTEM INITIALIZATION \/\/\/ */
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);		// slowing down ADC clock to 1MHz
	ADC_initGPIO();
	ADC_init();
	ADC_initDMA();
	MOTORS_initPOWERSUPPLY();				// 8V motor buck converter
	MOTORS_initGPIO();						// Motors + driver
	MOTORS_initPWM();						// PWM channels
	GPIO_SetBits(GPIOB, GPIO_Pin_7);		// Motor driver ON (PB7 = 1), OFF (PB7 = 0)
	BTM_init();								// Bluetooth UART
	LED_INIT();								// LEDs
	SysTick_Config( 8000000/1000 );			// 1ms system interrupt
	/* /\/\/\ SYSTEM INITIALIZATION /\/\/\ */

	/* Start ADC1 conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);


	/* DEFINICJA ZMIENNYCH ULOTNYCH */
	PID_initSTRUCTURE();
	flag = 0;
	flag_mode = 1;				//Domyslnie ustawiona w tryb manualny
	flag_mode_source = 0;
	flag_motor_ctrl = 0;
	flag_pid_ctrl = 0;
	/* Parametry istotne w trybie autonomicznym */
	Kp = 1;	  Kd = 1;	  Ki = 1;
	base_speed = 50;


	/* Main loop */
	while (1)
	{
		if( a % 500 == 0){
			static uint16_t Battery_voltage = 0;
			Battery_voltage = ADC_battery();
			if( Battery_voltage < 3000)
				GPIO_SetBits(GPIOB, GPIO_Pin_12);
			else
				GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		}

		/* TRYB AUTONOMICZNY */
		if( a % 10 == 0 ){	// 100 Hz control loop
			if( Flag_Start == 1){
				SENSOR_ProcessData(PID_Struct.Threshold);
				MOTOR_set(PID_Struct.BaseSpeed, PID_Struct.BaseSpeed);
			}
			else
				MOTOR_set( 0, 0);

			/* TRYB MANUALNY */
			if(flag_motor_ctrl == 1 && ((analyzed_frame[2] !=0) | (analyzed_frame[3] !=0))){
				MOTOR_set(analyzed_frame[2], analyzed_frame[3]);
			}
		}
	}
}


/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

