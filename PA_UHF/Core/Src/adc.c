/*
 * adc.c
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */
#include "adc.h"

void adc_init(ADC_t *adc) {

	for (int i = 0; i < CH_NUM; i++)
		adc->sum[i] = 0;
	adc->samples = 0;
}

void adc_media_movil_calc(ADC_t *adc) {
// TODO Revisar acá
	for (int i = 0; i < CH_NUM; i++) {

		if(adc->samples == 0)
			adc->sum[i] = adc->sum[i] - adc->read[i][SAMPLES-1] + adc->dma[i];
		else
			adc->sum[i] = adc->sum[i] - adc->read[i][adc->samples-1] + adc->dma[i];

		adc->read[i][adc->samples] = adc->dma[i];

		adc->media[i] = adc->dma[i];
	}
}

void adc_samples_update(ADC_t *adc) {
	adc->samples++;
	if (adc->samples >= SAMPLES)
		adc->samples = 0;
}

uint8_t adc_gain_calc(uint16_t adc_gain) {

	if (adc_gain >= 3781)
		return 45;
	else if (adc_gain < 3781 && adc_gain >= 1515)
		return 0.0022f * adc_gain + 36.6571f;
	else if (adc_gain < 1515 && adc_gain >= 1188)
		return (0.0153f * adc_gain + 16.8349f);
	else if (adc_gain < 1188 && adc_gain >= 1005)
		return (0.0273f * adc_gain + 2.540f);
	else if (adc_gain < 1005 && adc_gain >= 897)
		return (0.0463f * adc_gain - 16.5278f);
	else if (adc_gain < 897 && adc_gain >= 825)
		return (0.0694f * adc_gain - 37.2917f);
	else if (adc_gain < 825 && adc_gain >= 776)
		return (0.1020f * adc_gain - 64.1837f);
	else if (adc_gain < 776 && adc_gain >= 746)
		return (0.1667f * adc_gain - 114.333f);
	else if (adc_gain < 746 && adc_gain >= 733)
		return (0.3846f * adc_gain - 276.9231f);
	else if (adc_gain < 733 && adc_gain >= 725)
		return (0.625f * adc_gain - 453.125f);
	else if (adc_gain < 725)
		return 0;
	return 0;
}
