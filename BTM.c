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
			BTM_dataparse();
			MODE_HANDLER();
		}
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
