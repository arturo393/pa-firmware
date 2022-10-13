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






