/*
 * ltel.h
 *
 *  Created on: 27-09-2022
 *      Author: sigmadev
 */

#include "main.h"
#include "stdbool.h"
#include "math.h"

#ifndef INC_LTEL_H_
#define INC_LTEL_H_

typedef enum MODULE_FUNCTION {
	LOW_NOISE_AMPLIFIER = 0x08, POWER_AMPLIFIER = 0x09,
} Function_t;

typedef enum MODULE_ID {
	ID0 = 0x00, ID8 = 0x08, ID9 = 0X09
} Id_t;

typedef struct MODULE {
	uint8_t att;
	uint8_t gain;
	int8_t pout;
	int8_t pr;
	uint8_t voltage;
	int8_t pin;
	uint8_t current;
	bool state;
	float  temperature;
	float vswr;
	Id_t id;
	Function_t function;
}  Module_t;



static const uint8_t MODULE_ADDR = 0x08;
static const uint8_t MODULE_FUNCTION = 0x09;

static const uint8_t LTEL_START_MARK = 0x7e;
static const uint8_t LTEL_END_MARK = 0x7f;

void  module_init(Module_t*,Function_t,Id_t);
void module_calc_parameters(Module_t m,uint16_t* media_array);
float module_vswr_calc(int8_t pf, int8_t pr);


#endif /* INC_LTEL_H_ */
