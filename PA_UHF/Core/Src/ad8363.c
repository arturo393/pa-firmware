/*
 * ad8363.c
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */

#include "ad8363.h"

uint8_t ad8363_get_dbm(AD8363_t *ad,uint16_t value) {

	float m = (float) (AD8363_DBM_MAX - AD8363_DBM_MIN)
			/ (float) (ad->max - ad->min);
	float b = AD8363_DBM_MAX -ad->max * m;

	if (value > ad->max) {
		return AD8363_DBM_MAX;
	} else if (value < ad->min) {
		return AD8363_DBM_MIN;
	}
	return (int8_t) (m * (float) value + b);
}

bool ad8363_check_calibration(uint8_t value){
	return value != AD8363_IS_CALIBRATED_OK ? true: false;
}
