/*
 * rs485.h
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_

#include "main.h"
#include "stdbool.h"
#include "module.h"
#include "uart1.h"
#include "stdio.h"

typedef enum RS485_CMD {
	NONE,
	QUERY_PARAMETER_LTEL = 0x11,
	QUERY_PARAMETER_SIGMA,
	QUERY_PARAMETER_STR,
	QUERY_ADC,
	QUERY_ATT,
	SET_ATT_LTEL = 0x020,
	SET_POUT_MAX,
	SET_POUT_MIN,
	SET_PIN_MAX,
	SET_PIN_MIN,
	SET_VSWR_MAX,
	SET_VSWR_MIN,
	SET_ENABLE_PA = 0x22
} Rs485_cmd_t;

typedef enum RS485_STATUS{
	DATA_OK,
	NOT_VALID_FRAME,
	NO_VALID_MODULE,
	WRONG_MODULE_ID,
	CRC_ERROR,
	NO_DATA,
	WAITING
}Rs485_status_t ;

typedef struct RS485{
	Rs485_cmd_t cmd;
	uint8_t len;
	uint8_t *frame;
	Rs485_status_t status;
}RS485_t;

Rs485_status_t rs485_check_frame(RS485_t *r, UART1_t *u);
void rs485_set_query_frame(RS485_t* , Module_t *module);

#endif /* INC_RS485_H_ */
