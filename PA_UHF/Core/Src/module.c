/*
 * ltel.c
 *
 *  Created on: 27-09-2022
 *      Author: sigmadev
 */

#include <module.h>

const uint8_t MODULE_ADDR = 0x08;
const float ADC_CURRENT_FACTOR = 298.1818182f;
const float ADC_VOLTAGE_FACTOR = 0.007404330f;
const uint8_t MCP4706_CHIP_ADDR = 0b01100000;
const float REFERENCE_VOLTAGE = 3.3f;

POWER_AMPLIFIER_t* paInit() {
	POWER_AMPLIFIER_t *p = malloc(sizeof(POWER_AMPLIFIER_t));
	if (p != NULL) {
		p->function = POWER_AMPLIFIER;
		p->id = MODULE_ADDR;
		p->att = 0;
		p->gain = 0;
		p->pin = 0;
		p->pout = 0;
		p->temperature = 0;
		p->enable = 1;
		p->calc_en = 1;
		p->adc = malloc(sizeof(ADC_t));
		memset(p->adc->adcSum, 0, sizeof(p->adc->adcSum));
		memset(p->adc->adcSum, 0, sizeof(p->adc->adcMA));
		memset(p->adc->adcCounter, 0, sizeof(p->adc->adcCounter));
		for (int adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {
			memset(p->adc->adcReadings[adcIdx], 0,
					sizeof(p->adc->adcReadings[adcIdx]));
		p->attenuator = malloc(sizeof(BDA4601_t));

		}





	}
	return (p);

}

void startTimer3(uint8_t seconds) {
	/*enable clock access to timer 2 */
	SET_BIT(RCC->APBENR1, RCC_APBENR1_TIM3EN);
	/*set preescaler value */
	TIM3->PSC = 6400 - 1; // 64 000 000 / 64 00 = 1 000 000
	/* set auto-reload */
	TIM3->ARR = 1 * 5000 - 1; // 1 000  000 /
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);
	/* clear counter */
	TIM3->CNT = 0;
	/*enable timer 3*/
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);
	SET_BIT(TIM3->DIER, TIM_DIER_UIE);
	NVIC_EnableIRQ(TIM3_IRQn);
	CLEAR_BIT(TIM3->SR, TIM_SR_UIF);
}

void module_pa_state_update(POWER_AMPLIFIER_t *pa) {
	if (pa->enable == ON) {
		if (pa->temperature_out > MAX_TEMPERATURE)
			pa_off();
		if (pa->temperature_out < SAFE_TEMPERATURE) {
			pa_on();
			if (pa->vswr > MAX_VSWR)
				pa_off();
			if (pa->vswr < MAX_VSWR)
				pa_on();
		}
	}
	if (pa->enable == OFF)
		pa_off();
}

void processReceivedSerial(POWER_AMPLIFIER_t *p) {
	UART_t *serial = p->serial;
	UART_HandleTypeDef *handler = serial->handler;
	uint8_t *dataReceived = serial->data;
	uint8_t *len = &(serial->len);

	*len = handler->RxXferSize - handler->RxXferCount;

	if (dataReceived[*len - 1] != RDSS_END_MARK)
		return;
	if (dataReceived[0] != RDSS_START_MARK) {
		memset(dataReceived, 0, UART_SIZE);
		return;
	}

	// Data validation
	if (*len < MINIMUM_FRAME_LEN
			|| dataReceived[MODULE_TYPE_INDEX] != POWER_AMPLIFIER
			|| checkCRCValidity(dataReceived, *len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != p->id
			|| dataReceived[CMD_INDEX] == 0) {
		serial->handler->RxXferCount = UART_SIZE;
		handler->pRxBuffPtr = handler->pRxBuffPtr - *len;
		memset(serial->data, 0, UART_SIZE);
		serial->len = 0;
		return;
	}

	*len = exec(p, dataReceived);
	// Send response via UART
	HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(serial->handler, dataReceived, *len, 100);
	HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_RESET);

	serial->handler->RxXferCount = UART_SIZE;
	handler->pRxBuffPtr = handler->pRxBuffPtr - *len;
	memset(dataReceived, 0, *len);
	*len = 0;
}

uint8_t exec(POWER_AMPLIFIER_t *pa, uint8_t *dataReceived) {

	uint8_t dataLen = 0;
	uint8_t *response = dataReceived;
	uint8_t *responsePtr;
	uint8_t blank = 0x00;
	switch (response[CMD_INDEX]) {
	case QUERY_PARAMETER_LTEL:
		dataLen = sizeof(pa->enable) + sizeof(blank) + sizeof(pa->temperature)
				+ sizeof(pa->gain) + sizeof(pa->vswr) + sizeof(pa->att)
				+ sizeof(pa->pout) + sizeof(pa->pin);
		responsePtr = &dataReceived[DATA_START_INDEX];
		responsePtr = &dataReceived[DATA_START_INDEX];
		memcpy(responsePtr, &(pa->enable), sizeof(pa->enable));
		responsePtr += sizeof(pa->enable);
		memcpy(responsePtr, &blank, sizeof(blank));
		responsePtr += sizeof(blank);
		memcpy(responsePtr, &(pa->temperature), sizeof(pa->temperature));
		responsePtr += sizeof(pa->temperature);
		memcpy(responsePtr, &(pa->gain), sizeof(pa->gain));
		responsePtr += sizeof(pa->gain);
		memcpy(responsePtr, &(pa->vswr), sizeof(pa->vswr));
		responsePtr += sizeof(pa->vswr);
		memcpy(responsePtr, &(pa->att), sizeof(pa->att));
		responsePtr += sizeof(pa->att);
		memcpy(responsePtr, &(pa->pout), sizeof(pa->pout));
		responsePtr += sizeof(pa->pout);
		memcpy(responsePtr, &(pa->att), sizeof(pa->att));
		responsePtr += sizeof(pa->att);
		memcpy(responsePtr, &(pa->pin), sizeof(pa->pin));
		responsePtr += sizeof(pa->pin);
		break;
	case QUERY_PARAMETER_STR:
//		print_parameters(&uart1, pa);
		break;
	case QUERY_ADC:
//		print_adc(&uart1, adc.media);
		break;
	case SET_ATT_LTEL:
		pa->att = dataReceived[DATA_START_INDEX];
		bda4601_set_att(pa->att, 3);
//		m24c64_write_N(BASE_ADDR,  &(pa->att), ATT_VALUE_ADDR, 1);
//		sprintf((char*) uart1.tx_buffer, "Attenuation %u\r\n", pa->att);
//		uart1_send_frame((char*) uart1.tx_buffer, TX_BUFFLEN);
		break;
	case SET_ENABLE_PA:
		pa->enable = dataReceived[DATA_LENGHT2_INDEX];
		break;
	default:
		// Comando no reconocido, responder con un mensaje de error o realizar otra acción
	}

	uint8_t CRC_INDEX = DATA_START_INDEX + dataLen;
	uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
	dataReceived[DATA_LENGHT2_INDEX] = dataLen;
	dataReceived[END_INDEX] = LTEL_END_MARK;
	setCrc(dataReceived, DATA_START_INDEX + dataLen);
	return (END_INDEX + 1);
}

uint8_t readEepromData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector) {
    I2C_HandleTypeDef *i2c = p->i2c;
    BDA4601_t *attenuator = p->attenuator;
    uint8_t result = 0; // Initialize result to 0 (success).

    switch (sector) {
        case ATTENUATION:
            attenuator->val = readByte(i2c, M24C64_PAGE_ADDR(0), ATTENUATION_OFFSET);
            if (attenuator->val < MIN_DB_VALUE  || attenuator->val > MAX_DB_VALUE) {
            	attenuator->val = 0;
                result = 1; // Set result to 1 (error).
            }
            break;
        default:
            result = 2; // Set result to 2 (unsupported sector).
            break;
    }
    return (result);
}


void print_parameters(UART1_t *u, POWER_AMPLIFIER_t *pa) {
	sprintf((char*) u->tx_buffer,
			"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
			pa->pout, pa->att, pa->gain, pa->pin, pa->current,
			(uint8_t) pa->voltage);
	uart1_send_frame((char*) u->tx_buffer, TX_BUFFLEN);
	uart1_clean_buffer(u);
}

float vswr_calc(int8_t pf, int8_t pr) {

	float den;
	float num;
	float factor;
	float result;

	factor = (float) pf / (float) pr;
	den = 1.0f + sqrtf(factor);
	num = 1.0f - sqrtf(factor);
	result = den / num;
	return (result);
}

float arduino_map_float(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int8_t arduino_map_int8(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void rs485_set_query_frame(POWER_AMPLIFIER_t *pa) {

	uint8_t *response = pa->serial->data;
	uint8_t crc_frame[2];
	uint16_t crc;
	response[0] = LTEL_START_MARK;
	response[1] = pa->function;
	response[2] = pa->id;
	response[3] = pa->serial->data[CMD_INDEX];

	switch (pa->serial->data[CMD_INDEX]) {
	case QUERY_PARAMETER_LTEL:
		response[4] = 0x00;
		response[5] = 0x08;
		response[6] = pa->enable;
		response[7] = 0x00;
		response[8] = pa->temperature;
		response[9] = pa->gain;
		response[10] = pa->vswr;
		response[11] = pa->att;
		response[12] = pa->pout;
		response[13] = pa->pin;

		break;
	case QUERY_PARAMETER_SIGMA:
		response[4] = 0x00;
		response[5] = 0x08;
		response[6] = pa->enable;
		response[7] = 0x00;
		response[8] = pa->temperature;
		response[9] = pa->gain;
		response[10] = pa->vswr;
		response[11] = pa->att;
		response[12] = pa->pout;
		response[13] = pa->pin;
	default:
		response[0] = 0;
	}
	crc = crc_get(&(response[1]), 10);
	memcpy(crc_frame, &crc, 2);
	response[13 + 1] = crc_frame[0];
	response[13 + 2] = crc_frame[1];
	response[13 + 3] = LTEL_END_MARK;

}
