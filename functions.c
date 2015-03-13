/*
 * functions.c
 *
 *  Created on: Feb 14, 2014
 *      Author: Grzegorz
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "functions.h"

void SysTick_Handler(void)
{
	a++;
	LED_BLINK();
	if( a >= 1000 ){
		a = 0;
	}

}


void LED_INIT(void){

	/* Zasilenie portu B i D*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Inicjalizacja pinow do ktorych podpiete sa LEDy
	 * PB12 - LED4 (czerwona)
	 * PB13 - LED3 (zielona)
	 * PB14 - LED2 (niebieska spod)
	 * PB15 - LED1 (niebieska spod)
	 *
	 * LEDY z plytki z czujnikami
	 * PD1 - right LED
	 * PB2 - left LED
	 */

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PD1 pin remap */
	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	GPIO_SetBits(GPIOD, GPIO_Pin_1);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12 ); 		// Red LED off
}


/* Mruganie diodami */
void LED_BLINK(void){

}

/* Analiza ramki, zamiana tablicy charow na tablice int
 * ramka rozpoczyna sie %, konczy <LF>
 * przyklad otrzymanej ramki:
 * %,3,-1,-7,500
 * po przetworzeniu otrzymamy:
 * ramka[0] = 36	// to odpowiada znakowi %
 * ramka[1] = 3
 * ramka[2] = -1
 * ramka[3] = -7
 * ramka[4] = 500
 * ramka[5] - 10	// to odpowiada znakowi LF
 */
void BTM_dataparse(void){

	/* Liczniki pomocnicze */
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t k = 0;
	/* Pomocnicza tablica charow */
	unsigned char frame[10];


		/* Ramka rozpoczyna sie znakiem '%' */
	for( i = 0; i < 30; i ++){
			if( received_frame[i] == '%' ){
				i = 0;
				j = 0;
				k = 0;
				analyzed_frame[k] = received_frame[i];
				k++;
			}

			/* Analizujemy elementy ramki pomiedzy przecinkami i skladamy je w jeden element */
			if( ( received_frame[i] != ',' ) & (received_frame[i] != '%' ) ){
				frame[j] = received_frame[i];
				j++;
			}

			/* W przypadku wykrycia przecinka przypisz go ramce */
			if( (( (received_frame[i] == ',') | (received_frame[i] == 10) ) & (analyzed_frame[0] == '%')) ){
				if( i > 1 ){
					analyzed_frame[k] = atoi(frame);
					k++;
				}

					/* Zerowanie pomocniczej ramki */
					for( j = 0; j < 10; j++){
						frame[j] = 0;
					}
					j = 0;
					if( received_frame[i] == 0x0A){
						analyzed_frame[k] = received_frame[i];
						break;
					}
			}
	}
}

/* Funkcja ta obsluguje otrzymywane ramki danych i decyduje o wyborze poszczegolnych trybow pracy Linefollowera */
void MODE_HANDLER(void){
	/* Funkcja ta WYMAGA zdeklarowania zmiennych volatile w pliku FUNCTIONS.H:
	 * 	volatile uint8_t flag_mode;			// Wybor trybu, flag_mode = 1 (manual), flag_mode = 2 (autonomous)
		volatile uint8_t flag_mode_source;	// Wybor zrodla flag_mode_source =1 (PC), flag_mode_source = 2 (SMARTPHONE)
		volatile uint8_t flag_motor_ctrl;	// Flaga obslugi silnikow
		volatile uint8_t flag_pid_ctrl;		// Flaga obslugi parametrow regulatora PID

		volatile uint8_t Kp, Kd, Ki;		// Parametry regulatora PID
		volatile uint16_t base_speed;		// Bazowa predkosc silnikow (istotna w trybie autonomicznym)
	 */
	if( flag == 1 ){
		BTM_dataparse();
		switch ( analyzed_frame[1] ) {
			case 1:
				flag_mode = analyzed_frame[2]; // flag_mode == 0 (manual mode), flag_mode == 1 (autonomous mode)
				break;
			case 2:
				flag_motor_ctrl = 1;
				break;
			case 3:
				flag_pid_ctrl = 1;
				break;
			default:
				flag_motor_ctrl = 0;
				flag_pid_ctrl = 0;
				break;
		}
		flag = 0;
	}


	/* TRYB MANUALNY */
	/* Obsluga ramki danych dotyczacej silnikow */
	if( flag_motor_ctrl == 1){
		/* Wlaczanie i wylaczanie silnikow mozliwe w oby dwoch trybach */
		if( analyzed_frame[4] == 0 )
			GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		if( analyzed_frame[4] == 1 )
			GPIO_SetBits(GPIOB, GPIO_Pin_7);
		/* Kontroluj silniki tylko w trybie manualnym */
		if( flag_mode == 1 ){
			MOTOR_set(analyzed_frame[2], analyzed_frame[3]);
		}

		flag_motor_ctrl = 0;
	}

	/* TRYB AUTONOMICZNY */
	if( flag_mode == 2 ){
		if( flag_pid_ctrl == 1 ){
			Kp = analyzed_frame[2];
			Kd = analyzed_frame[3];
			Ki = analyzed_frame[4];
			base_speed = analyzed_frame[5];
			flag_pid_ctrl = 0;
		}

		MOTOR_set(base_speed, base_speed);
	}
}
