/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**	Author		: Grzegorz Wojcik
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "functions.h"
#include "MOTORS.h"
#include "BTM.h"
#include "STM32vldiscovery.h"

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
  int i = 0;

  /* TODO - Add your application code here */

  a = 0;	// Ustawianie wartosci poczatkowej zmiennych ulotnych
  s = 0;
  /* INICJALIZACJA PERYFERIOW */
  SysTick_Config( SystemCoreClock/1000 );	// Licznik systemowy
  //LED_INIT();								// Diody LED
  MOTORS_init();							// Silniki + mostek TB6612
  PWM_init();								// Kanaly PWM
  TRANSFORMATOR_INIT();						// Przetwornica 8V
  BTM_init();								// Bluetooth

  GPIO_SetBits(GPIOB, GPIO_Pin_7);			// Wlaczenie mostka (PB7 = 1), wylaczenie (PB7 = 0)

  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDOn(LED3);

  s = SystemCoreClock;

  	  /* DEFINICJA ZMIENNYCH ULOTNYCH */
	  flag = 0;
	  flag_mode = 1;				//Domyslnie ustawiona w tryb manualny
	  flag_mode_source = 0;
	  flag_motor_ctrl = 0;
	  flag_pid_ctrl = 0;
	  /* Parametry istotne w trybie autonomicznym */
	  Kp = 1;	  Kd = 1;	  Ki = 1;
	  base_speed = 50;

	  /* Zerowanie bufora */
	  for( i = 0; i < 30; i++ ){
		  received_frame[i] = 0;
	  }

  /* Infinite loop */
  while (1)
  {
	  /* OBSLUGA RAMEK DANYCH */
	  MODE_HANDLER();
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

/*
 	if( flag == 1 ){
		analyze_frame();
		if( analyzed_frame[1] == 1 ){
			flag_mode = analyzed_frame[2];
		}
		if( analyzed_frame[1] == 2 ){
			flag_motor_ctrl = 1;
		}
		flag = 0;
	}
 */

