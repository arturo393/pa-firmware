/*
 * led.c
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */
#include "led.h"

void led_init(void){

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

}

void led_enable_kalive(uint32_t counter){
	if (HAL_GetTick() - counter > LED_KA_STATE_TIMEOUT)
		counter = HAL_GetTick();
	else {
		if (HAL_GetTick() - counter > LED_KA_ON_TIMEOUT)
			sys_rp_led_off();
		else
			sys_rp_led_on();
	}
}
void led_reset(LED_t *l){
l->ch_counter = 0;
l->cl_counter = 0;
l->cn_counter = 0;
l->ka_counter = 0;
l->sysrp_counter = 0;
l->th_counter = 0;
l->tok_counter = 0;
}
