/*
 * ADC.c
 *
 *  Created on: Jun 16, 2014
 *      Author: Grzegorz
 */


#include <stddef.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "ADC.h"


#include "ADC.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include <misc.h>


void ADC_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitStructure.ADC_Mode = ADC_Mode_FastInterl;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Tryb ciagly, napiecie mierzone przez caly czas, a nie jednokrotnie
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Wlaczenie skanowania wszystkich kanalow. Po zakonczeniu pomiaru jednego kanalu, nastapi natychmiastowy pomiar kanalu nastepnego
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Wyrownanie danych do prawej
	ADC_InitStructure.ADC_NbrOfChannel = 14; // Liczba uzytych kanalow = 13
	ADC_Init(ADC1, &ADC_InitStructure); // Inicjalizacja ADC1

	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5); // SENSOR 00 - PC0 - CHANNEL 10
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_239Cycles5); // SENSOR 01 - PC1 - CHANNEL 11
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_239Cycles5); // SENSOR 02 - PC2 - CHANNEL 12
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,4,ADC_SampleTime_239Cycles5); // SENSOR 03 - PC3 - CHANNEL 13
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,5,ADC_SampleTime_239Cycles5); // 	SENSOR 04 - PA0 - CHANNEL 00
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,6,ADC_SampleTime_239Cycles5); // 	SENSOR 05 - PA1 - CHANNEL 01
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,7,ADC_SampleTime_239Cycles5); // 	SENSOR 06 - PA2 - CHANNEL 02
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,8,ADC_SampleTime_239Cycles5); // 	SENSOR 07 - PA3 - CHANNEL 03
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,9,ADC_SampleTime_239Cycles5); // 	SENSOR 08 - PA4 - CHANNEL 04
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,10,ADC_SampleTime_239Cycles5); // SENSOR 09 - PA5 - CHANNEL 05
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,11,ADC_SampleTime_239Cycles5); // SENSOR 10 - PA6 - CHANNEL 06
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,12,ADC_SampleTime_239Cycles5); // SENSOR 11 - PA7 - CHANNEL 07
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,13,ADC_SampleTime_239Cycles5); // SENSOR 12 - PC4 - CHANNEL 14

	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,14,ADC_SampleTime_239Cycles5); // wybor (zworka) pomiedzy sharpem a pomiarem aku

	ADC_Cmd(ADC1, ENABLE); // Wlaczenie ADC1

	ADC_DMACmd(ADC1, ENABLE);
}

void ADC_initGPIO(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void ADC_initDMA(void)
{

	#define ADC1_DR_Address 0x4001244C; // Adres rejestru ADC1 DR - STM32F10XX_ADC.C -> CDR_ADDRESS
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address; 	// Adres docelowy transferu: rejestr ADC1_DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned long int)&buforADC; 	// Adres poczatku bloku do przeslania
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 						// Kierunek transferu. Dla STM32F10xx: DMA_DIR_PeripheralSRC
	DMA_InitStructure.DMA_BufferSize = 14; 									// Liczba elementow do przeslania (dlugosc bufora)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 		// Wylaczenie automatycznego zwiekszania adresu po stronie pamieci (bufora)
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 				// Wylaczenie automatycznego zwiekszania adresu po stronie pamieci (bufora)
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // Rozmiar pojedynczych przesylanych danych po stronie licznika. HalfWord = 16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 						// Tryb dzialania kontrolera DMA - powtarzanie cykliczne
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 					// Priorytet DMA - wysoki
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure); 	// Zapis konfiguracji
	DMA_Cmd(DMA1_Channel1, ENABLE); 				// Wlacz kanal 0
}

void ADC_BatteryMonitor(void){

	static uint8_t i = 0;
	static uint16_t ADC_tab[3] = {0};
	static uint16_t voltage = 0;

	if( i < 3 ){
			ADC_tab[i] = buforADC[13];
			i++;
	}
	else{
		ADC_tab[0] = ADC_tab[1];
		ADC_tab[1] = ADC_tab[2];
		ADC_tab[2] = buforADC[13];
	}

	voltage = (ADC_tab[0] + ADC_tab[1] + ADC_tab[2])/3;
	if( voltage < 3000)
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

