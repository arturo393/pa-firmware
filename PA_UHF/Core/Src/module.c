/*
 * ltel.c
 *
 *  Created on: 27-09-2022
 *      Author: sigmadev
 */

#include <module.h>

void  module_init(Module_t* module ,Function_t funcion, Id_t id){
module->function = funcion;
module->id = id;
module->att = 0;
module->gain =   0;
module->pin = 0;
module->pout = 0;
module->temperature = 0;
module->state = true;
module_sample_timer3_init();
pa_off();

/* PA3  PA_HAB as output - ENABLE - DISABLE PA */
SET_BIT(GPIOA->MODER, GPIO_MODER_MODE3_0);
CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE3_1);

}

float module_vswr_calc(int8_t pf, int8_t pr){

	float den;
	float num;
	float factor;
	float result;

	factor = (float) pf/ (float) pr;
	den  = 1.0f + sqrtf(factor);
	num = 1.0f - sqrtf(factor);
	result = den / num;
	return result;
}

void module_sample_timer3_init() {
	/*enable clock access to timer 2 */
	SET_BIT(RCC->APBENR1, RCC_APBENR1_TIM3EN);
	/*set preescaler value */
	TIM3->PSC = 6400 - 1; // 64 000 000 / 64 00 = 1 000 000
	/* set auto-reload */
	TIM3->ARR = 10000 - 1; // 1 000  000 /
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);
	/* clear counter */
	TIM3->CNT = 0;
	/*enable timer 3*/
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);
	SET_BIT(TIM3->DIER, TIM_DIER_UIE);
	NVIC_EnableIRQ(TIM3_IRQn);
	CLEAR_BIT(TIM3->SR, TIM_SR_UIF);
}
