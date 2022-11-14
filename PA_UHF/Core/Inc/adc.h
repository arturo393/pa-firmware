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

#define SAMPLES  20
#define CH_NUM 7

typedef enum ADC_INDEX {
	GAIN_i, CURRENT_i, VOLTAGE_i, VSWR_i, POUT_i, PIN_i, TEMP_i
} ADC_INDEX_t;

static const float ADC_CURRENT_FACTOR = 298.1818182f;
static const float ADC_VOLTAGE_FACTOR = 0.007404330f;

typedef struct ADC_t {
	volatile uint16_t dma[CH_NUM];
	uint16_t read[CH_NUM][SAMPLES];
	uint16_t media[CH_NUM];
	int32_t sum[CH_NUM];
	uint8_t samples;
	bool is_dma_ready;
} ADC_t;

#define adc_start_conversion() SET_BIT(ADC1->CR,ADC_CR_ADSTART)
void adc_init(ADC_t* adc);
void adc_samples_update(ADC_t *adc);
uint8_t adc_gain_calc(uint16_t adc_gain);
void adc_media_movil_calc(ADC_t *adc);

#endif /* INC_ADC_H_ */
