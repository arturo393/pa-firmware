/*
 * rs485.c
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */
#include <rdss.h>

uint16_t crc_get(uint8_t* buffer, uint8_t buff_len) {
	uint8_t b;
	uint8_t i;
	uint16_t generator = 0x1021; //divisor is 16bit
	uint16_t crc = 0;			 // CRC value is 16bit

	for (b = 0; b < buff_len; b++) {
		crc ^= ((uint16_t) (buffer[b] << 8)); // move byte into MSB of 16bit CRC
		for (i = 0; i < 8; i++) {
			if ((crc & 0x8000) != 0) // test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			else
				crc <<= 1;
		}
	}
	return (crc);
}

RDSS_status_t checkCRCValidity(uint8_t *frame, uint8_t len) {
	uint16_t calculatedCrc;
	uint16_t savedCrc;
	savedCrc = ((uint16_t) frame[len - 2] << 8);
	savedCrc |= (uint16_t) frame[len - 3];
	calculatedCrc = crc_get(&frame[1], len - 4);
	return ((calculatedCrc == savedCrc) ? DATA_OK : CRC_ERROR);
}

uint8_t setCrc(uint8_t *buff, uint8_t size) {
	uint8_t crc_frame[2];
	uint16_t crc;
	crc = crc_get(buff + 1, size - 1);
	memcpy(crc_frame, &crc, 2);
	buff[size++] = crc_frame[0];
	buff[size++] = crc_frame[1];
	return (2);
}
