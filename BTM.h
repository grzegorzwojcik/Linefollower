/*
 * BTM.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Grzegorz
 */

#ifndef BTM_H_
#define BTM_H_

void BTM_init(void);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void analyze_frame(void);
char* itoa(int value, char* buffer, int base);

#endif /* BTM_H_ */
