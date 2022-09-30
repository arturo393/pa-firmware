/*
 * max4003.h
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */

#ifndef INC_MAX4003_H_
#define INC_MAX4003_H_
#include "main.h"
#include "stdbool.h"



#define MAX4003_DBM_MAX 0
#define MAX4003_DBM_MIN -30
#define MAX4003_ADC_MAX 1883
#define MAX4003_ADC_MIN 488

#define MAX4003_IS_CALIBRATED_OK 0xAA

typedef struct  MAX4003{
uint16_t max;
uint16_t min;
} MAX4003_t;


uint8_t max4003_get_dbm(MAX4003_t *mx,uint16_t value) ;
bool max4003_check_calibration(uint8_t value);


#endif /* INC_MAX4003_H_ */
