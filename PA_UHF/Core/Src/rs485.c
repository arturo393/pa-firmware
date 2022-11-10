/*
 * rs485.c
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */
#include "rs485.h"

void rs485_init(RS485_t *r) {
	r->len = 0;
	r->status = DONE;
	r->cmd = NONE;
}
Rs485_status_t rs485_check_frame(RS485_t *r, UART1_t *u) {

	if (u->rx_count > (3 + 1 + 2)){
		if (u->rx_buffer[0] == LTEL_START_MARK)
			if(u->rx_buffer[u->rx_count-1] == LTEL_END_MARK)
				return VALID_FRAME;
			else
				return START_READING;
		else
			return START_READING;
	}
	return  WAITING;
}


Rs485_status_t  rs485_check_valid_module(UART1_t *uart1){
if (uart1->rx_buffer[1] == POWER_AMPLIFIER) {
			if (uart1->rx_buffer[2] == ID8) {
				for (int i = 3; i  < uart1->rx_count; i++)
					if (uart1->rx_buffer[i] == LTEL_END_MARK)
						return   DATA_OK;
			} else
				return   WRONG_MODULE_ID;
		} else
			return  NO_VALID_MODULE;
return NO_VALID_MODULE;
}

void rs485_set_query_frame(RS485_t *r, Module_t *module) {

	uint8_t crc_frame[2];
	uint16_t crc;
	r->frame[0] = LTEL_START_MARK;
	r->frame[1] = module->function;
	r->frame[2] = module->id;
	r->frame[3] = r->cmd;

	if (module->function == LOW_NOISE_AMPLIFIER) {
		switch (r->cmd) {
		case QUERY_PARAMETER_LTEL:
			r->frame[4] = 0x00;
			r->frame[5] = 0x05;
			r->frame[6] = 0x00;
			r->frame[7] = module->att;
			r->frame[8] = module->gain;
			r->frame[9] = module->pout;
			r->frame[10] = module->voltage;
			break;
		case QUERY_PARAMETER_SIGMA:
			r->frame[4] = 0x06;
			r->frame[5] = module->pout;
			r->frame[6] = module->att;
			r->frame[7] = module->gain;
			r->frame[8] = module->current;
			r->frame[9] = module->voltage;
			r->frame[10] = module->pin;
		default:
			r->frame[0] = 0;
		}
		crc = crc_get(&(r->frame[1]), 10);
		memcpy(crc_frame, &crc, 2);
		r->frame[11] = crc_frame[0];
		r->frame[12] = crc_frame[1];
		r->frame[13] = LTEL_END_MARK;

	} else if (module->function == POWER_AMPLIFIER) {

		switch (r->cmd) {
		case QUERY_PARAMETER_LTEL:
			r->frame[4] = 0x00;
			r->frame[5] = 0x08;
			r->frame[6] = module->state;
			r->frame[7] = 0x00;
			r->frame[8] = module->temperature;
			r->frame[9] = module->gain;
			r->frame[10] = module->vswr;
			r->frame[11] = module->att;
			r->frame[12] = module->pout;
			r->frame[13] = module->pin;

			break;
		case QUERY_PARAMETER_SIGMA:
			r->frame[4] = 0x00;
			r->frame[5] = 0x08;
			r->frame[6] = module->state;
			r->frame[7] = 0x00;
			r->frame[8] = module->temperature;
			r->frame[9] = module->gain;
			r->frame[10] = module->vswr;
			r->frame[11] = module->att;
			r->frame[12] = module->pout;
			r->frame[13] = module->pin;
		default:
			r->frame[0] = 0;
		}
		crc = crc_get(&(r->frame[1]), 10);
		memcpy(crc_frame, &crc, 2);
		r->frame[13 + 1] = crc_frame[0];
		r->frame[13 + 2] = crc_frame[1];
		r->frame[13 + 3] = LTEL_END_MARK;
	}
}

void rs485_parameters_cmd_action(uint8_t *frame) {

}

void rs485_update_status_by_uart(RS485_t *rs485,UART1_t *uart1){

	switch (rs485->status) {
	case DATA_OK:
		rs485->cmd = uart1->rx_buffer[3];
		rs485->status = DONE;
		break;
	case START_READING:
		rs485->status = WAITING;
		if (uart1_clean_by_timeout(uart1, "START_READING"))
			rs485->status = DONE;
		break;
	case VALID_FRAME:
		rs485->status = rs485_check_valid_module(uart1);
		break;
	case NOT_VALID_FRAME:
		// TODO
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case WRONG_MODULE_ID:
		// TODO
		uart1_clean_buffer(uart1);
		rs485->status = DONE;
		break;
	case CRC_ERROR:
		// TODO add crc
		uart1_clean_buffer(uart1);
		break;
	case WAITING:
		rs485->status = rs485_check_frame(rs485, uart1);
		uart1_clean_by_timeout(uart1, "WAITING");
		break;
	case DONE:
		uart1_send_str("DONE\r\n");
		uart1_clean_buffer(uart1);
		rs485->status = WAITING;
		break;
	default:
		rs485->status = DONE;
		uart1_clean_buffer(uart1);
		break;
	}
}
