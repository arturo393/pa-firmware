/*
 * max4003.c
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */
#include "max4003.h"

uint8_t max4003_get_dbm( MAX4003_t *mx,uint16_t value) {

	float m = (float) ( MAX4003_DBM_MAX -  MAX4003_DBM_MIN)
			/ (float) (mx->max - mx->min);
	float b =  MAX4003_DBM_MAX -mx->max * m;

	if (value > mx->max) {
		return  MAX4003_DBM_MAX;
	} else if (value < mx->min) {
		return  MAX4003_DBM_MIN;
	}
	return (int8_t) (m * (float) value + b);
}

bool  max4003_check_calibration(uint8_t value){
	return value !=  MAX4003_IS_CALIBRATED ? true: false;
}
