/*
 eeprom.c * bda4601.c
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */
#include "bda4601.h"

void bda4601_init(uint8_t att) {

	bda4601_pin_init();
	if (att > MIN_DB_VALUE && att < MAX_DB_VALUE)
		bda4601_set_initial_att(att, STARTING_MILLIS);
	else
		bda4601_set_att(MIN_DB_VALUE, TIMES);
}

void bda4601_pin_init(void) {

	/* DATA_ATT  PB1 as output */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE1_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE1_1);
	/* CLK_ATT PB0 as output */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE0_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE0_1);
	/* LE_ATT PA7 as output*/
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE7_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE7_1);

}

void bda4601_set_att(uint8_t val, uint8_t times) {

	if (val < MIN_DB_VALUE || val > MAX_DB_VALUE) {
		val = MIN_DB_VALUE;
	}
	val *= 2;
	for (uint8_t i = 0; i < times; i++) {
		uint8_t mask = 0b00100000;
		for (uint8_t j = 0; j < 6; j++) {
			//Ciclo for de 6 vueltas para enviar los 6bits de configuración
			if (mask & val) {
				//Si el bit de la mascara en 1 coincide con el bit del valor, entonces
				SET_BIT(GPIOB->MODER, GPIO_MODER_MODE1_0);
				//HAL_GPIO_WritePin(GPIOA, DATA_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin data en alto
			} else {
				//	HAL_GPIO_WritePin(GPIOA, DATA_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin data en bajo
				CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE1_0);
			}

			SET_BIT(GPIOB->MODER, GPIO_MODER_MODE0_0);
			//HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin clock en alto
			HAL_Delay(1); //Delay de 1mS
			CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE0_0);
			//HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin clock en bajo
			mask = mask >> 1; //Muevo la máscara una posición
		}
		SET_BIT(GPIOA->MODER, GPIO_MODER_MODE7_0);
		//HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin LE en alto
		HAL_Delay(1);
		CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE7_0);
		//HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin LE en bajo
	}
}

void bda4601_set_initial_att(uint8_t value, uint16_t period_millis) {
	uint16_t t_step = 500;
	uint16_t times = period_millis / t_step;
	int att_step = value / times;
	int att = 0;

	for (int i = 0; i <= times; i++) {
		bda4601_set_att(att, 2);
		att += att_step;
		if (att >= MAX_DB_VALUE) {
			return;
		}
		HAL_Delay(t_step);
	}
}

void setInitialAttenuation(BDA4601_t *b, uint16_t millis) {
	uint16_t t_step = 500;
	uint16_t times = millis / t_step;
	int att_step = b->val / times;
	int att = 0;

	for (int i = 0; i <= times; i++) {
		setAttenuation(b);
		att += att_step;
		if (att >= MAX_DB_VALUE) {
			return;
		}
		HAL_Delay(t_step);
	}
}

void setAttenuation(BDA4601_t *b) {
	uint8_t val = b->val;
	if (val < MIN_DB_VALUE || val > MAX_DB_VALUE) {
		val = MIN_DB_VALUE;
	}
	val *= 2;
	for (uint8_t i = 0; i < TIMES; i++) {
		uint8_t mask = 0b00100000;
		for (uint8_t j = 0; j < 6; j++) {
			if (mask & b->val)
				HAL_GPIO_WritePin(b->dataPort, b->dataPin, GPIO_PIN_SET); //Pin data en alto
			 else
				HAL_GPIO_WritePin(b->dataPort, b->dataPin, GPIO_PIN_RESET); //Pin data en bajo

			HAL_GPIO_WritePin(b->clkPort, b->clkPin, GPIO_PIN_SET); //Pin clock en alto
			HAL_Delay(1); //Delay de 1mS
			HAL_GPIO_WritePin(b->clkPort, b->clkPin, GPIO_PIN_RESET); //Pin clock en bajo
			mask = mask >> 1; //Muevo la máscara una posición
		}
		HAL_GPIO_WritePin(b->lePort, b->lePin, GPIO_PIN_SET); //Pin LE en alto
		HAL_Delay(1);
		HAL_GPIO_WritePin(b->lePort, b->lePin, GPIO_PIN_RESET); //Pin LE en bajo
	}
}
