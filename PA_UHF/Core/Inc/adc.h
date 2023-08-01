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

#define adc_start_conversion() SET_BIT(ADC1->CR,ADC_CR_ADSTART)
#define SAMPLES  20
#define ADC_WINDOW_SIZE 20

extern const float ADC_CURRENT_FACTOR;
extern const float ADC_VOLTAGE_FACTOR;


typedef enum {
	GAIN_CH, CURRENT_CH, VOLTAGE_CH, PREF_CH, POUT_CH, PIN_CH, TEMPERATURE_CH,ADC_CHANNELS
} ADC_CHANNEL_t;


typedef struct ADC_t {
	uint32_t adcValues[ADC_CHANNELS];
	uint8_t adcCounter[ADC_CHANNELS];
	uint16_t adcReadings[ADC_CHANNELS][ADC_WINDOW_SIZE];
	uint16_t adcMA[ADC_CHANNELS];
	uint16_t adcSum[ADC_CHANNELS];
} ADC_t;


void adc_init(ADC_t* adc);
void adc_samples_update(ADC_t *adc);
uint8_t adc_gain_calc(uint16_t adc_gain);
void adc_media_movil_calc(ADC_t *adc);

#endif /* INC_ADC_H_ */
