/*
 * adc.h
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "stdbool.h"
#include "stdlib.h"

#define adc_start_conversion() SET_BIT(ADC1->CR,ADC_CR_ADSTART)
#define SAMPLES  20
#define ADC_WINDOW_SIZE 50

extern const float ADC_CURRENT_FACTOR;
extern const float ADC_VOLTAGE_FACTOR;

typedef enum {
	GAIN_CH,
	CURRENT_CH,
	VOLTAGE_CH,
	PREF_CH,
	POUT_CH,
	PIN_CH,
	ADC_CHANNELS
} ADC_CHANNEL_t;

typedef enum {
	ADC_WAITING, ADC_TIMEOUT
} ADC_Status_t;

typedef struct ADC_t {
	uint32_t values[ADC_CHANNELS];
	uint8_t adcCounter[ADC_CHANNELS];
	uint16_t adcReadings[ADC_CHANNELS][ADC_WINDOW_SIZE];
	uint16_t ma[ADC_CHANNELS];
	uint16_t adcSum[ADC_CHANNELS];
	ADC_HandleTypeDef *handler;
	ADC_Status_t status;
	ADC_TypeDef *reg;
} ADC_t;

ADC_t* adcInit(ADC_TypeDef *reg);
void configureADC(ADC_t *adc);
void readADC(ADC_t* adc);
void configADC(ADC_TypeDef *reg);
void movingAverage(ADC_t *adc);
#endif /* INC_ADC_H_ */
