/*
 * adc.c
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */
#include "adc.h"

void configureADC(ADC_t *adc) {
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // Enable GPIOA clock

	GPIOA->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2
			| GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6; // Analog mode for pins PA0, PA1, PA2, PA4, PA5, PA6

	RCC->APBENR2 |= RCC_APBENR2_ADCEN;  // Enable ADC clock

	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;  // Select HCLK as ADC clock

	ADC1->CR |= ADC_CR_ADVREGEN;  // Enable ADC voltage regulator

	for (uint32_t i = 0; i < 1000; i++) // Wait for ADC voltage regulator start-up time
		__NOP();

	ADC1->CR |= ADC_CR_ADCAL;  // Start calibration
	while (ADC1->CR & ADC_CR_ADCAL)
		;  // Wait for calibration to complete

	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;  // Disable continuous conversion

	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0; // Enable external trigger (software trigger)
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTSEL; // Select external trigger source as SWSTART

	ADC1->CFGR1 |= ADC_CFGR1_RES_1;  // Set ADC resolution to 12 bits

	// Set slowest sampling time (160.5 ADC cycles) for all channels
	ADC1->SMPR = ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2
			| ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6; // Select channels to be converted

	// Enable end of conversion interrupt
	ADC1->IER |= ADC_IER_EOCIE;
	NVIC_EnableIRQ(ADC1_IRQn);  // Enable ADC interrupt in NVIC

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
		;  // Wait for ADC ready
}

void performADCRead(ADC_t *adc) {

	SET_BIT(adc->reg->ISR, ADC_ISR_EOSEQ);
	adc->reg->CR |= ADC_CR_ADSTART; /* Start the ADC conversion */
	uint16_t ISR_EOC = 0;
	uint64_t ISER_EOC_MAX = HAL_MAX_DELAY;
	int i = 0;
	do {
		while ((adc->reg->ISR & ADC_ISR_EOC) == 0 && adc->status != ADC_TIMEOUT) /* Wait end of conversion */
		{
			/* For robust implementation, add here time-out management */
			if (ISR_EOC > ISER_EOC_MAX)
				adc->status = ADC_TIMEOUT;
			else
				ISR_EOC++;
		}
		adc->values[i] = adc->reg->DR; /* Store the ADC conversion result */
		i++;
	} while ((adc->reg->ISR & ADC_ISR_EOSEQ) == 0 && i < ADC_CHANNELS);
	SET_BIT(adc->reg->ISR, ADC_ISR_EOSEQ);
	adc->status = ADC_WAITING;
	return;
}

void readADC(ADC_t *adc) {
	int i = 0;
	uint16_t ISR_EOC = 0;
	uint16_t ISER_EOC_MAX = 6000;
	while (i < ADC_CHANNELS) {
		if (i < 3) {
			adc->reg->CHSELR = (1 << i);
		} else {
			adc->reg->CHSELR = (1 << (i + 1));
		}
		adc->reg->SMPR = ADC_SMPR_SMP1_0 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_2; // slowest sampling time
		adc->reg->CR |= ADC_CR_ADSTART; // start conversion
		while ((adc->reg->ISR & ADC_ISR_EOC) == 0) { // wait end of conversion
			if (ISR_EOC > ISER_EOC_MAX) {
				// add here time-out management
				break;
			} else {
				ISR_EOC++;
			}
		}
		adc->values[i] = adc->reg->DR; // store the ADC conversion result
		i++;
	}
}

ADC_t* adcInit(ADC_TypeDef *reg) {
	ADC_t *a = malloc(sizeof(ADC_t));
	if (a != NULL) {
		memset(a->adcSum, 0, sizeof(a->adcSum));
		memset(a->ma, 0, sizeof(a->ma));
		memset(a->adcCounter, 0, sizeof(a->adcCounter));
		for (int adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {
			memset(a->adcReadings[adcIdx], 0,
					sizeof(a->adcReadings[adcIdx]));
		}
		a->reg = ADC1;
	}
	return (a);
}

void configADC(ADC_TypeDef *reg) {
	RCC->APBENR2 |= RCC_APBENR2_ADCEN; // enable ADC clock
	reg->CR &= ~ADC_CR_ADEN; // disable ADC
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
		; // wait for ADC to be disabled
	reg->CFGR1 &= ~ADC_CFGR1_RES; // 12-bit resolution
	reg->CFGR1 &= ~ADC_CFGR1_ALIGN; // right alignment
	reg->CFGR2 &= ~ADC_CFGR2_CKMODE; // select HSI16 as clock source
	reg->SMPR = 7; // slowest sampling time
	reg->CHSELR = (ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2
			| ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6); // select channels 0, 1, 2, 4, 5 and 6
	reg->IER = 0; // disable interrupts
	reg->CR |= ADC_CR_ADEN; // enable ADC
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
		; // wait for ADRDY flag to be set
}

void movingAverage(ADC_t *adc) {
	for (int adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {
		// Subtract oldest value from the sum
		adc->adcSum[adcIdx] -=
				adc->adcReadings[adcIdx][adc->adcCounter[adcIdx]];

		// Subtraction overflow check
		if ((int32_t) adc->adcSum[adcIdx] < 0)
			adc->adcSum[adcIdx] = 0;

		// Store the new value in the buffer
		adc->adcReadings[adcIdx][adc->adcCounter[adcIdx]] = adc->values[adcIdx];

		// Add new value to the sum
		adc->adcSum[adcIdx] += adc->values[adcIdx];

		// Increment the current index and wrap around if necessary
		adc->adcCounter[adcIdx]++;
		if (adc->adcCounter[adcIdx] >= ADC_WINDOW_SIZE) {
			adc->adcCounter[adcIdx] = 0;
		}

		// Calculate and return the average
		adc->ma[adcIdx] = adc->adcSum[adcIdx] / ADC_WINDOW_SIZE;
	}
}

