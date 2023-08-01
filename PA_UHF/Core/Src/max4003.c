/*
 * max4003.c
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */
#include "max4003.h"

bool max4003_check_calibration(uint8_t value) {

	return value != MAX4003_IS_CALIBRATED ? true : false;
}

