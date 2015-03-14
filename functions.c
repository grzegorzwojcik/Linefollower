/*
 * functions.c
 *
 *  Created on: Feb 14, 2014
 *      Author: Grzegorz
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"

#include "functions.h"
#include "BTM.h"
#include "CONTROLLER.h"

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



void USART1_IRQHandler(void){

	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; 	// the character from the USART6 data register is saved in t

		if(t != '\n' && cnt < MAX_STRLEN){	// sprawdzamy czy pojawiajacy sie znak nie jest znakiem konca linii
			// oraz sprawdzamy czy liczba znaków odbieranych nie przekracza wielkosci bufora MAX_STRLEN
			BUFFOR[cnt] = t;
			cnt++;
		}
		else{ // jesli pojawi sie znak konca linii lub bufor jest przepelniony wowczas zeruje licznik odebranych znakow i wysylamy znaki znajdujace sie w buforze

			/* W momencie, gdy nowoodebrany ciag znakow jest krotszy od poprzednio
			odebranego nalezy wykasowac smieci pozostale w buforze	*/
			for(cnt = cnt; cnt < sizeof(BUFFOR); cnt++){
				BUFFOR[cnt] = NULL;
			}
			cnt = 0;
			BTM_DataParse();
			BTM_ProcessBuffor();
		}
	}
}

