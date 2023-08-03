/*
 * led.c
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */
#include "led.h"

LED_INFO_t** ledInit(uint8_t ch) {

	LED_INFO_t **l = malloc(sizeof(LED_INFO_t) * ch);
	if (l != NULL) {

	}

	return (l);
}

void led_init(LED_t *led) {

	/*CURRENT LOW LED PA12  as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE12_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE12_1);

	/*CURRENT NORMAL LED PA11  as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE11_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE11_1);

	/*CURRENT HIGH  LED PA10  as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE10_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE10_1);

	/*SYS_RP LED PC6  as output */
	SET_BIT(GPIOC->MODER, GPIO_MODER_MODE6_0);
	CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODE6_1);

	/* TEMPERATURE OK  LED PA9  as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE9_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE9_1);

	/* TEMPERATURE HIGH LED PA8  as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE8_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE8_1);

	led_reset(led);

}

void led_off(void) {

}

void kaUpdate(LED_INFO_t *l) {
	if (HAL_GetTick() - l->startMillis > LED_KA_STATE_TIMEOUT)
		l->startMillis = HAL_GetTick();
	else {
		if (HAL_GetTick() - l->startMillis > LED_KA_ON_TIMEOUT)
			HAL_GPIO_WritePin(l->port, l->pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(l->port, l->pin, GPIO_PIN_RESET);
	}
}

void led_reset(LED_t *l) {
	l->ch_counter = 0;
	l->cl_counter = 0;
	l->cn_counter = 0;
	l->ka = HAL_GetTick();
	l->sysrp_counter = 0;
	l->th_counter = 0;
	l->tok_counter = 0;
	current_low_led_on();
	current_normal_led_on();
	current_high_led_on();
	sys_rp_led_on();
	temperature_ok_led_on();
	temperature_high_led_on();
}

void currentUpdate(LED_INFO_t **l, int16_t current) {
	HAL_GPIO_WritePin(l[CURRENT_LOW]->port, l[CURRENT_LOW]->pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(l[CURRENT_NORMAL]->port, l[CURRENT_NORMAL]->pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(l[CURRENT_HIGH]->port, l[CURRENT_HIGH]->pin,
			GPIO_PIN_SET);
	if (current >= LED_MAX_CURRENT) {
		HAL_GPIO_WritePin(l[CURRENT_HIGH]->port, l[CURRENT_HIGH]->pin,
				GPIO_PIN_RESET);
	} else if (current > LED_MIN_CURRENT && current < LED_MAX_CURRENT) {
		HAL_GPIO_WritePin(l[CURRENT_NORMAL]->port, l[CURRENT_NORMAL]->pin,
				GPIO_PIN_RESET);
	} else if (current <= LED_MIN_CURRENT) {
		HAL_GPIO_WritePin(l[CURRENT_LOW]->port, l[CURRENT_LOW]->pin,
				GPIO_PIN_RESET);
	}
}

void temperatureUpdate(LED_INFO_t **l, uint8_t temperature) {
	HAL_GPIO_WritePin(l[TEMPERATURE_OK]->port, l[TEMPERATURE_OK]->pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(l[TEMPERATURE_HIGH]->port, l[TEMPERATURE_HIGH]->pin,
			GPIO_PIN_SET);
	if (temperature > LED_MAX_TEMPERATURE)
		HAL_GPIO_WritePin(l[TEMPERATURE_HIGH]->port, l[TEMPERATURE_HIGH]->pin,
				GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(l[TEMPERATURE_OK]->port, l[TEMPERATURE_OK]->pin,
				GPIO_PIN_RESET);

}

void led_current_update(int16_t current) {

	if (current >= LED_MAX_CURRENT) {
		current_high_led_on();
		current_normal_led_off();
		current_low_led_off();
	} else if (current > LED_MIN_CURRENT && current < LED_MAX_CURRENT) {
		current_high_led_off();
		current_normal_led_on();
		current_low_led_off();

	} else if (current <= LED_MIN_CURRENT) {
		current_high_led_off();
		current_normal_led_off();
		current_low_led_on();
	}
}
