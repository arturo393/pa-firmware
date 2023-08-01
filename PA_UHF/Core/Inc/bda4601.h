/*
 * bda4601.h
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */

#ifndef INC_BDA4601_H_
#define INC_BDA4601_H_
#include "main.h"
#include "stdbool.h"

#define MAX_DB_VALUE 31
#define MIN_DB_VALUE 0
#define STARTING_MILLIS 5000U
#define D
#define TIMES 3

typedef struct{
	uint16_t dataPin;
	GPIO_TypeDef *dataPort;
	uint16_t clkPin;
	GPIO_TypeDef *clkPort;
	uint16_t lePin;
	GPIO_TypeDef *lePort;
	uint8_t val;
}BDA4601_t;

void bda4601_init(uint8_t att);
void bda4601_pin_init(void);
void bda4601_set_att(uint8_t, uint8_t);
void bda4601_set_initial_att(uint8_t, uint16_t );
void setAttenuation(BDA4601_t *b);
void setInitialAttenuation(BDA4601_t *b, uint16_t millis);
#endif /* INC_BDA4601_H_ */
