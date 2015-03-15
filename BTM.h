/*
 * BTM.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Grzegorz
 */
#include <stddef.h>
#include "stm32f10x.h"
#include "CONTROLLER.h"

#ifndef BTM_H_
#define BTM_H_

#define MAX_STRLEN 35
volatile char BUFFOR[MAX_STRLEN];
extern volatile uint16_t buforADC[14];

/* FLagi poszczegolnych czynnosci */
volatile uint8_t Flag_Start; 		// Flag_Start to flaga uruchamiajaca (= 1) lub zatrzymujaca (= 0) robota w trybie autonomicznym,
volatile uint8_t flag_mode;			// Wybor trybu, flag_mode = 1 (manual), flag_mode = 2 (autonomous)
volatile uint8_t flag_mode_source;	// Wybor zrodla flag_mode_source =1 (PC), flag_mode_source = 2 (SMARTPHONE)
volatile uint8_t flag_motor_ctrl;	// Flaga obslugi silnikow
volatile uint8_t flag_pid_ctrl;		// Flaga obslugi parametrow regulatora PID
volatile uint8_t flag_apk;			// Flaga obslugi zadan wysylanych z poziomu aplikacji

volatile uint8_t Kp, Kd, Ki;		// Parametry regulatora PID
volatile uint16_t base_speed;		// Bazowa predkosc silnikow (istotna w trybie autonomicznym)


volatile unsigned char received_frame[30];		/* Bufor, odebrana ramka danych */
volatile int16_t analyzed_frame[20];			/* Przeanalizowana ramka danych */

/**
  * @brief  Bluetooth GPIO and UART initialization
  * @param  None
  * @retval None
  */
void BTM_init(void);
/**
  * @brief  Sending a string s via USARTx
  * @param  None
  * @retval None
  */
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
/**
  * @brief  Data parsing from volatile buffer
  * @param  None
  * @retval None
  */
void BTM_DataParse(void);
void BTM_ProcessBuffor(void);

char* itoa(int value, char* buffer, int base);

#endif /* BTM_H_ */
