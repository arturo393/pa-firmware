
/*
 * ad8363.h
 *
 *  Created on: Sep 29, 2022
 *      Author: sigmadev
 */
#include "main.h"
#include "stdbool.h"

#define AD8363_DBM_MAX 0
#define AD8363_DBM_MIN -30
#define AD8363_ADC_MAX ((uint16_t) 1882)
#define AD8363_ADC_MIN  ((uint16_t) 487)

#define AD8363_IS_CALIBRATED 0xAA

typedef struct AD8363{
uint16_t max;
uint16_t min;
} AD8363_t;


uint8_t ad8363_get_dbm(AD8363_t *ad,uint16_t value) ;
bool ad8363_check_calibration(uint8_t value);
