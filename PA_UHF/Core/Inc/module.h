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

#define MAX_TEMPERATURE 30 // 75
#define SAFE_TEMPERATURE 25

#define pa_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD3)
#define pa_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD3)
#define pa_state()  READ_BIT(GPIOA->ODR,GPIO_ODR_OD3) ? 1 : 0

typedef enum MODULE_FUNCTION {
	LOW_NOISE_AMPLIFIER = 0x08, POWER_AMPLIFIER = 0x09,
} Function_t;

typedef enum MODULE_ID {
	ID0 = 0x00, ID8 = 0x08, ID9 = 0X09
} Id_t;

typedef enum MODULE_S{
	OFF,
	ON
}State_t;
typedef struct MODULE {
	uint8_t att;
	uint8_t gain;
	int8_t pout;
	int8_t pr;
	uint8_t voltage;
	int8_t pin;
	uint16_t current;
	State_t state;
	float  temperature;
	float temperature_out;
	float vswr;
	Id_t id;
	Function_t function;

	uint8_t temperature_high;
}  Module_t;





static const uint8_t MODULE_ADDR = 0x08;
static const uint8_t MODULE_FUNCTION = 0x09;

static const uint8_t LTEL_START_MARK = 0x7e;
static const uint8_t LTEL_END_MARK = 0x7f;

void  module_init(Module_t*,Function_t,Id_t);
void module_calc_parameters(Module_t m,uint16_t* media_array);
float module_vswr_calc(int8_t pf, int8_t pr);
void module_sample_timer3_init();

#endif /* INC_LTEL_H_ */
