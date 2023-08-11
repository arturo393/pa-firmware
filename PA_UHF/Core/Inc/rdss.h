/*
 * rs485.h
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */

#ifndef INC_RDSS_H_
#define INC_RDSS_H_

#include <pa.h>
#include "main.h"
#include "stdbool.h"
#include "uart1.h"
#include "stdio.h"
#include "string.h"

#define LTEL_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14


#define LTEL_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14
#define LTEL_START_MARK 0x7e
#define RDSS_END_MARK 0x7f
#define RDSS_START_MARK 0x7e
#define LTEL_END_MARK  0x7f
#define RDSS_BUFFER_SIZE 25
#define CRC_SIZE 2
#define LTEL_SET_LENGTH  13
#define LTEL_QUERY_LENGTH  9
#define MINIMUM_FRAME_LEN 6
#define ATTENUATION_VALUE_INDEX 5
#define QUERY_STATUS_BUFFER_SIZE 34

typedef enum MODULE_FUNCTION {
	SERVER,
	QUAD_BAND,
	PSU,
	TETRA,
	ULADR,
	VLADR,
	BDA,
	LOW_NOISE_AMPLIFIER,
	POWER_AMPLIFIER = 0x09,
	UHF_TONE,
	SNIFFER
} Function_t;

typedef enum RS485_CMD {
	NONE,
	QUERY_MODULE_ID = 0x10,
	QUERY_PARAMETER_LTEL = 0x11,
	QUERY_PARAMETER_SIGMA,
	QUERY_PARAMETER_STR,
	QUERY_ADC,
	QUERY_ATT,
	SET_ATT_LTEL = 0x20,
	SET_ENABLE_PA = 0x22,
	SET_POUTLEVEL,
	SET_POUT_MAX,
	SET_POUT_MIN,
	SET_PIN_MAX,
	SET_PIN_MIN,
	SET_VSWR_MAX,
	SET_VSWR_MIN,

	QUERY_PARAMETER_PdBm,
} Rs485_cmd_t;

typedef enum RS485_STATUS {
	DATA_OK,
	START_READING,
	VALID_FRAME,
	NOT_VALID_FRAME,
	WRONG_MODULE_FUNCTION,
	WRONG_MODULE_ID,
	CRC_ERROR,
	NO_VALID_MODULE,
	DONE,
	WAITING,
	VALID_MODULE,
	CHECK_LORA_DATA,
	LORA_RECEIVE,
	LORA_SEND,
	UART_SEND,
	UART_VALID
} RDSS_status_t;

typedef enum RS485_i {
	START_INDEX,
	MODULE_TYPE_INDEX,
	MODULE_ID_INDEX,
	CMD_INDEX,
	DATA_LENGHT1_INDEX,
	DATA_LENGHT2_INDEX,
	DATA_START_INDEX
} Rs485_i;

typedef struct {
	uint8_t *frame;
	Rs485_cmd_t cmd;
	uint8_t len;
	uint8_t *buff;
	uint8_t buffSize;
	uint16_t crcCalculated;
	uint16_t crcReceived;
	uint8_t idQuery;
	uint8_t idReceived;
	uint8_t id;
	RDSS_status_t status;
	uint8_t queryBuffer[QUERY_STATUS_BUFFER_SIZE];
	uint32_t lastUpdateTicks;
} RDSS_t;
uint16_t crc_get(uint8_t* buffer, uint8_t buff_len);
RDSS_status_t checkCRCValidity(uint8_t *frame, uint8_t len);
uint8_t setCrc(uint8_t *buff, uint8_t size);
#endif /* INC_RS485_H_ */
