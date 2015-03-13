/*
 * functions.h
 *
 *  Created on: Feb 14, 2014
 *      Author: Grzegorz
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

volatile int a,s;
volatile uint8_t flag;

/* FLagi poszczegolnych czynnosci */
volatile uint8_t flag_mode;			// Wybor trybu, flag_mode = 1 (manual), flag_mode = 2 (autonomous)
volatile uint8_t flag_mode_source;	// Wybor zrodla flag_mode_source =1 (PC), flag_mode_source = 2 (SMARTPHONE)
volatile uint8_t flag_motor_ctrl;	// Flaga obslugi silnikow
volatile uint8_t flag_pid_ctrl;		// Flaga obslugi parametrow regulatora PID

volatile uint8_t Kp, Kd, Ki;		// Parametry regulatora PID
volatile uint16_t base_speed;		// Bazowa predkosc silnikow (istotna w trybie autonomicznym)


volatile unsigned char received_frame[30];		/* Bufor, odebrana ramka danych */
volatile int16_t analyzed_frame[20];			/* Przeanalizowana ramka danych */


void LED_INIT(void);
void LED_BLINK(void);
void MODE_HANDLER(void);

#endif /* FUNCTIONS_H_ */
