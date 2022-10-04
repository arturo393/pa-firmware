/*
 * rs485.c
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */
#include "rs485.h"

Rs485_cmd_t rs485_check_frame(uint8_t *frame, uint8_t len) {

	if (strlen(frame) > (3 + 1 + 2))
		if (frame[0] == LTEL_START_MARK)
			if (frame[1] == MODULE_FUNCTION)
				if (frame[2] == MODULE_ADDR)
					for (int i = 3; i < len; i++)
						if (frame[i] == LTEL_END_MARK)
							return true;
	return frame[3];
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
		r->frame[13+1] = crc_frame[0];
		r->frame[13+2] = crc_frame[1];
		r->frame[13+3] = LTEL_END_MARK;
	}
}

void rs485_parameters_cmd_action(uint8_t *frame) {

}