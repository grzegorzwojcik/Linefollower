/*
 * Project: LINEFOLLOWER
 * MOTORS.c
 *
 *  Created on: Feb 15, 2014
 *      Author: Grzegorz
 */

#include <stddef.h>
#include "stm32f10x.h"

#include "MOTORS.h"


/*** W PLIKU MAIN NALEZY DOLACZYC BIBLIOTEKE #include "MOTORS.h"
 *	W funkcji glownej nalezy zamiescic "MOTORS_init(void)", a nastepnie "PWM_init(void)"
 * 	silniki wowczas zostana zainicjalizowane, jednakze ich kontrola wymaga wlaczenia mostka,
 * 	oraz sterowania odpowiednimi pinami
 *
 * 	Sterowanie predkoscia silnikow (dwa kanaly OC3 (PC8) i OC4(PC9) odbywa sie przy uzyciu komendy:
 * 	"TIM3 -> CCR(x)" = wypelnienie;
 * 	gdzie X to numer kanalu (3 lub 4), a wypelnienie to zmienna int z zakresu 0-655
 * 	czyli np.TIM3 -> CCR4 = 665;
 * 	timer zlicza w gore, tak wiec w tej sytuacji mamy maksymalne wypelnienie, w przypadku
 * 	TIM3 -> CCR4 = 0 mamy zerowe wypelnienie kanalu 4 timera 3.
 * 	***/

/* Inicjalizacja przetwornicy 8V (ustawienie zworki na piny EN + uC
 * zwykle usawienie portow GPIO, ustawienie PC10 = 1 wlacza przetwornice 8V
 * 											PC10 = 0 wylaczenie przetwornicy
 * 		istnieje mozliwosc ustawienia zworki na piny V_IN + EN, wowczas przetwornica
 * 		jest caly czas wlaczona bez koniecznosci uzywania programu
 */
void MOTORS_initPOWERSUPPLY(void){
	/* Zasilenie portu B i D*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* PRZETWORNICA WLACZONA PRZY INICJALIZACJI */
	GPIO_SetBits(GPIOC, GPIO_Pin_10);
}

void MOTORS_initGPIO(void){

	/* Zasilenie portu B */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/* Inicjalizacja pinow do ktorych podpiete sa silniki */
	/* PB5, PB6 silnik prawy
	 * PB8, PB9 silnik lewy
	 * PB7 - PIN STANDBY, 	ustawienie stanu wysokiego (1) na tym pinie oznacza wlaczenie mostka TB6612
	 * 						ustawienie stanu niskiego (0) na tym pinie powoduje wylaczenie mostka TB6612
	 */
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Poczatkowa konfiguracja: mostek wylaczony (PB7=0); wlaczenie mostka (PB7=1);
	 * oby dwa silniki wylaczone (PB5 | PB6 | PB8 | PB9 = 0 )
	 * wlaczone kanaly PWM
	 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9);
}

void MOTORS_initPWM(void){

	/* wyliczenie wartosci prescalera */
	uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Aktywowanie (zasilenie) portu C */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		/* Inicjalizacja kanalow PWM
		 * PC9 - kanal PWM silnika PRAWEGO (TIM3_CH4)
		 * PC8 - kanal PWM silnika LEWEGO (TIM3_CH3)
		 */
		GPIO_InitTypeDef GPIO_InitStructure2;
		GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOC, &GPIO_InitStructure2);

	/* Musimy remapowac piny PC8 i PC9 na timer3 kanaly 3 i 4 */
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/* Aktywowanie (zasilenie) timera 3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		/* Inicjalizacja Timera TIM3 */
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = 665;
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Inicjalizacja kanalow 3(PC8 -OC3) timera 3 */
		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);

		TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* Inicjalizacja kanalow 4(PC9 -OC4) timera 3 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);

		TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/*** 	Odpalenie timera	***/
		TIM_ARRPreloadConfig(TIM3, ENABLE);
		TIM_Cmd(TIM3, ENABLE);

	/*** Poczatkowa konfiguracja: wlaczone kanaly PWM ***/
	GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);

	/*
	 	 * PC9 - kanal PWM silnika PRAWEGO (TIM3_CH4)
		 * PC8 - kanal PWM silnika LEWEGO (TIM3_CH3)
	 */
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
}

void MOTOR_set(int16_t MOT_LEFT, int16_t MOT_RIGHT){

	if( MOT_LEFT > 660 )
		MOT_LEFT = 660;
	if( MOT_RIGHT > 660 )
		MOT_RIGHT = 660;

	if( MOT_LEFT < -660 )
		MOT_LEFT = -660;
	if( MOT_RIGHT < -660 )
		MOT_RIGHT = -660;

	/* JEDZ DO PRZODU */
	if( MOT_LEFT > 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	}

	if( MOT_RIGHT > 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		}
	/* ZATRZYMAJ SILNIK */
	if( MOT_LEFT == 0)
		GPIO_SetBits(GPIOB, GPIO_Pin_9 | GPIO_Pin_8);
	if( MOT_RIGHT == 0)
		GPIO_SetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6);

	/* JEDZ W TYL */
	if( MOT_LEFT < 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	}
	if( MOT_RIGHT < 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	}

	if( TIM3->CCR3 != MOT_LEFT ){
		if( MOT_LEFT < 0 )
			MOT_LEFT = MOT_LEFT * (-1);
		TIM3->CCR3 = MOT_LEFT;
	}
	if( TIM3->CCR4 != MOT_RIGHT ){
		if( MOT_RIGHT < 0 )
			MOT_RIGHT = MOT_RIGHT * (-1);
		TIM3->CCR4 = MOT_RIGHT;
	}
	/* PRZYPISANIE WARTOSCI PWM ODPOWIEDNIM KANALOM */

}
