/*
 * BTM.c
 *
 *  Created on: Mar 3, 2014
 *      Author: Grzegorz
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "functions.h"
#include "BTM.h"



void BTM_init(void){
		GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as TX and RX
		USART_InitTypeDef USART_InitStructure; // this is for the USART6 initilization
		NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)


		/* Clock configuration -------------------------------------------------------*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		/* Configure the GPIO ports( USART1 Transmit and Receive Lines) */
		/* Configure the USART1_Tx as Alternate function Push-Pull */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure the USART1_Rx as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* USART1 configuration ------------------------------------------------------*/
		/* USART1 configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 interrupt */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1,USART_IT_TXE,ENABLE);


		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
		/* Here the USART1 receive interrupt is enabled
		 * and the interrupt controller is configured
		 * to jump to the USART1_IRQHandler() function
		 * if the USART1 receive interrupt occurs */


		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART6 interrupts
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART6 interrupts
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART6 interrupts are globally enabled
		NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

		// finally this enables the complete USART6 peripheral
		USART_Cmd(USART1, ENABLE);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

/*------------------------------------------------------------------------*//**
* \brief itoa() function.
* \details Converts signed integer to an char array. Valid 'base' in [2;16].
* Only base == 10 values are treated as signed
*
* \param [in] value the value which will be converted
* \param [out] buffer the output buffer
* \param [in] base the base of conversion
* \return pointer to \em buffer
*//*-------------------------------------------------------------------------*/
char* itoa(int value, char* buffer, int base)
{
	static const char digits[]="0123456789abcdef";

	char* buffer_copy=buffer;
	int32_t sign=0;
	int32_t quot,rem;

	if ((base>=2)&&(base<=16))				// is the base valid?
	{
		if (base == 10 && (sign = value) < 0)// negative value and base == 10? store the copy (sign)
			value = -value;					// make it positive

		do
		{
			quot=value/base;				// calculate quotient and remainder
			rem=value%base;
			*buffer++ = digits[rem];		// append the remainder to the string
		} while ((value=quot));				// loop while there is something to convert

		if (sign<0)							// was the value negative?
			*buffer++='-';					// append the sign

		__reverse(buffer_copy,buffer-1);		// reverse the string
	}

	*buffer='\0';
	return buffer_copy;
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
void BTM_DataParse(void){

	/* Liczniki pomocnicze */
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t k = 0;
	uint8_t Flag_Dalej = 0;
	/* Pomocnicza tablica charow */
	char frame[10];

		/* Ramka rozpoczyna sie znakiem '%' */
	for( i = 0; i < sizeof(BUFFOR); i ++){
		if( BUFFOR[i] == '%' ){
			i = 0;
			j = 0;
			k = 0;
			analyzed_frame[k] = BUFFOR[i];
			k++;
			Flag_Dalej = 1;
		}

		if(Flag_Dalej == 1)
		{
			/* Analizujemy elementy ramki pomiedzy przecinkami i skladamy je w jeden element */
			if( ( BUFFOR[i] != ',' ) & (BUFFOR[i] != '%' ) ){
				frame[j] = BUFFOR[i];
				j++;
			}

			/* W przypadku wykrycia przecinka przypisz go ramce */
			if( (( (BUFFOR[i] == ',') | (BUFFOR[i] == 10) ) & (BUFFOR[0] == '%')) ){
				if( i > 1 ){
				analyzed_frame[k] = atoi(frame);
				k++;
				}

				/* Zerowanie pomocniczej ramki */
				for( j = 0; j < 10; j++){
					frame[j] = 0;
				}
				j = 0;
				if( BUFFOR[i] == 0x0A){
					analyzed_frame[k] = BUFFOR[i];
					break;
				}
			}
		}
	}
}

/* Processing received data BUFFOR[] */
void BTM_ProcessBuffor(void){

	/* Funkcja ta WYMAGA zdeklarowania zmiennych volatile w pliku FUNCTIONS.H:
	  volatile uint8_t flag_mode;			// Wybor trybu, flag_mode = 1 (manual), flag_mode = 2 (autonomous)
		volatile uint8_t flag_mode_source;	// Wybor zrodla flag_mode_source =1 (PC), flag_mode_source = 2 (SMARTPHONE)
		volatile uint8_t flag_motor_ctrl;	// Flaga obslugi silnikow
		volatile uint8_t flag_pid_ctrl;		// Flaga obslugi parametrow regulatora PID
		volatile uint8_t flag_apk; 			// Flaga obslugi zadan wysylanych z poziomu aplikacji C#
	 */

	if(analyzed_frame[0] == 37) flag = 1;		// Instrukcja sprawdzajaca czy pierwszym elementem odczytanych wartosci jest '%'
		else flag = 0;

	if( flag == 1 ){
		switch ( analyzed_frame[1] ) {
			case 1:
				flag_mode = analyzed_frame[1]; // flag_mode == 0 (manual mode), flag_mode == 1 (autonomous mode)
				break;
			case 2:
				flag_mode = analyzed_frame[1];
				flag_motor_ctrl = 1;
				break;
			case 3:
				Flag_Start = 0;
				flag_mode = analyzed_frame[1];
				flag_pid_ctrl = 1;
				break;
			case 4:
				flag_mode = analyzed_frame[1];
				flag_apk = 1;
				flag_pid_ctrl = 0;
				flag_motor_ctrl = 0;
				break;
			default:
				flag_motor_ctrl = 0;
				flag_pid_ctrl = 0;
				break;
		}
		flag = 0;
	}


	/* C# APPLICATION SECTION */

	if( flag_mode == 2)
	{
		switch( analyzed_frame[4] ) {
			case 0:
				Flag_Start = 0;
				break;
			case 1:
				Flag_Start = 1;
				break;
		}
	}

	/* AUTONOMOUS MODE */
	if( flag_mode == 3 ){
		if( flag_pid_ctrl == 1 ){
			Flag_Start = 0;

			Kp = analyzed_frame[2];
			PID_Struct.Kp = analyzed_frame[2];

			Kd = analyzed_frame[3];
			PID_Struct.Kd = analyzed_frame[3];

			Ki = analyzed_frame[4];
			PID_Struct.Ki = analyzed_frame[4];

			base_speed = analyzed_frame[5];
			PID_Struct.BaseSpeed = analyzed_frame[5];

			flag_pid_ctrl = 0;
		}
	}

	/* SMARTPHONE SECTION */
	if( flag_mode == 4)
	{
		switch ( analyzed_frame[2] ) {
			case 0:		/* STOP THE ROBOT */
				Flag_Start = 0;
				break;
			case 1:		/* START THE ROBOT */
				Flag_Start = 1;
				break;

			case 2:		/* Sensor calibration */
				SENSOR_Calibration();
				break;

			case 3:		/* Send current PID parameters */
				break;
		}
	}
}
