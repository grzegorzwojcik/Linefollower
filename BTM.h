/*
 * BTM.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Grzegorz
 */
#include <stddef.h>
#include "stm32f10x.h"

#ifndef BTM_H_
#define BTM_H_

#define MAX_STRLEN 35
volatile char BUFFOR[MAX_STRLEN];

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
void BTM_dataparse(void);

char* itoa(int value, char* buffer, int base);

#endif /* BTM_H_ */
