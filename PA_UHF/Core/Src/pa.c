/*
 * ltel.c
 *
 *  Created on: 27-09-2022
 *      Author: sigmadev
 */

#include <pa.h>

const uint8_t MODULE_ADDR = 0x08;
const uint8_t MCP4706_CHIP_ADDR = 0b01100000;

const float POUT_REF[POUT_LEVEL_VALUES] = { 1.84, 2.01, 2.2, 2.35 };

// Lookup table for power calculation: pow_table[x] = 10^(x/20)
const float pow_table[] = { 1.000000f, 1.122018f, 1.258925f, 1.412538f,
		1.584893f, 1.778279f, 1.995262f, 2.238721f, 2.511886f, 2.818383f,
		3.162278f, 3.548134f, 3.981072f, 4.466836f, 5.011872f, 5.623413f,
		6.309573f, 7.079458f, 7.943282f, 8.912509f, 10.000000f };
const double epsilon = 1e-9; // Adjust the value based on your application's requirements

POWER_AMPLIFIER_t* paInit() {
	POWER_AMPLIFIER_t *p = malloc(sizeof(POWER_AMPLIFIER_t));
	if (p != NULL) {
		p->function = POWER_AMPLIFIER;
		p->id = MODULE_ADDR;
		p->gain = 0;
		p->pIn = 0;
		p->pOut = 0;
		p->temp = 0;
		p->enable = 1;
		p->status = PA_WAIT;
	}
	return (p);
}

void startTimer3(uint8_t seconds) {
	/*enable clock access to timer 2 */
	SET_BIT(RCC->APBENR1, RCC_APBENR1_TIM3EN);
	/*set preescaler value */
	TIM3->PSC = 6400 - 1; // 64 000 000 / 64 00 = 1 000 000
	/* set auto-reload */
	TIM3->ARR = seconds * 10000 - 1; // 1 000  000 /
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);
	/* clear counter */
	TIM3->CNT = 0;
	/*enable timer 3*/
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);
	SET_BIT(TIM3->DIER, TIM_DIER_UIE);
	NVIC_EnableIRQ(TIM3_IRQn);
	CLEAR_BIT(TIM3->SR, TIM_SR_UIF);
}

void tooglePa(POWER_AMPLIFIER_t *pa) {
	if (pa->enable == ON) {
		if ((HAL_GetTick() - pa->ouputMillis) > 1000) {

			if (pa->tempOut > MAX_TEMPERATURE)
				pa->ouputState = GPIO_PIN_RESET;

			if (pa->tempOut < SAFE_TEMPERATURE) {
				if (pa->pOut >= 5) {
					if (pa->vswr > MAX_VSWR) {
						pa->ouputState = GPIO_PIN_RESET;
						pa->status = POUT_VSWR_ALERT;
					} else {
						pa->ouputState = GPIO_PIN_SET;
						pa->status = PA_OK;
					}
				} else {
					pa->ouputState = GPIO_PIN_SET;
					pa->status = PA_OK;
				}

			}
		}
	}
	if (pa->enable == OFF)
		pa->ouputState = GPIO_PIN_RESET;

	if (pa->ouputState != pa->lastOutputState) {
		pa->lastOutputState = pa->ouputState;
		if (pa->ouputState == GPIO_PIN_SET)
			pa->ouputMillis = HAL_GetTick();
	}

	HAL_GPIO_WritePin(pa->port, pa->pin, pa->ouputState);
}

void processReceivedSerial(POWER_AMPLIFIER_t *p) {
	UART_t *u = p->serial;
	uint8_t *dataReceived = u->data;
	if (dataReceived[u->len - 1] != RDSS_END_MARK)
		return;
	if (dataReceived[0] != RDSS_START_MARK) {
		memset(dataReceived, 0, UART_SIZE);
		return;
	}

	// Data validation
	if (u->len < MINIMUM_FRAME_LEN
			|| dataReceived[MODULE_TYPE_INDEX] != POWER_AMPLIFIER
			|| checkCRCValidity(dataReceived, u->len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != p->id
			|| dataReceived[CMD_INDEX] == 0) {
		memset(u->data, 0, UART_SIZE);
		u->len = 0;
		return;
	}

	u->len = exec(p, dataReceived);

	// Send response via UART
	SET_BIT(u->dePort->ODR, u->dePin);
	uartSend(u);
	CLEAR_BIT(u->dePort->ODR, u->dePin);

	memset(dataReceived, 0, u->len);
	u->len = 0;
}

uint8_t exec(POWER_AMPLIFIER_t *pa, uint8_t *dataReceived) {
	UART_t *u = pa->serial;
	uint8_t dataLen = 0;
	uint8_t *response = dataReceived;
	uint8_t *responsePtr;
	uint8_t blank = 0x00;
	POUT_LEVEL_t pOutLevel;
	switch (response[CMD_INDEX]) {
	case QUERY_PARAMETER_LTEL:
		dataLen = sizeof(pa->enable) + sizeof(blank) + sizeof(pa->temp)
				+ sizeof(pa->gain) + sizeof(pa->vswr) + sizeof(pa->attenuator->val)
				+ sizeof(pa->pOut) + sizeof(pa->pIn);
		responsePtr = &dataReceived[DATA_START_INDEX];
		responsePtr = &dataReceived[DATA_START_INDEX];
		memcpy(responsePtr, &(pa->enable), sizeof(pa->enable));
		responsePtr += sizeof(pa->enable);
		memcpy(responsePtr, &blank, sizeof(blank));
		responsePtr += sizeof(blank);
		memcpy(responsePtr, &(pa->temp), sizeof(pa->temp));
		responsePtr += sizeof(pa->temp);
		memcpy(responsePtr, &(pa->gain), sizeof(pa->gain));
		responsePtr += sizeof(pa->gain);
		memcpy(responsePtr, &(pa->vswr), sizeof(pa->vswr));
		responsePtr += sizeof(pa->vswr);
		memcpy(responsePtr, &(pa->attenuator->val), sizeof(pa->attenuator->val));
		responsePtr += sizeof(pa->attenuator->val);
		memcpy(responsePtr, &(pa->pOut), sizeof(pa->pOut));
		responsePtr += sizeof(pa->pOut);
		memcpy(responsePtr, &(pa->pIn), sizeof(pa->pIn));
		responsePtr += sizeof(pa->pIn);
		break;
	case QUERY_PARAMETER_STR:
		printParameters(pa);
		break;
	case QUERY_ADC:
		printRaw(pa);
		break;
	case SET_ATT_LTEL:
		pa->attenuator->val = dataReceived[DATA_START_INDEX];
		setAttenuation(pa->attenuator);
		saveData(pa, ATTENUATION);
		break;
	case SET_ENABLE_PA:
		pa->enable = dataReceived[DATA_START_INDEX];
		break;
	case SET_POUTLEVEL:
		pOutLevel = dataReceived[DATA_START_INDEX];
		pa->status = setDacLevel(pa, pOutLevel);
		if (pa->status != PA_OK)
			dataReceived[DATA_START_INDEX] = 0xFF;
		pa->status = PA_WAIT;
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
	HAL_StatusTypeDef res;

	switch (sector) {
	case ATTENUATION:
		res = readPage(i2c, M24C64_PAGE_ADDR(0), &(attenuator->val),
		ATTENUATION_OFFSET, sizeof(attenuator->val));
		if (res != HAL_OK)
			attenuator->val = 0;
		if (attenuator->val < MIN_DB_VALUE || attenuator->val > MAX_DB_VALUE) {
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

HAL_StatusTypeDef saveData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector) {
	I2C_HandleTypeDef *i2c = p->i2c;
	BDA4601_t *a = p->attenuator;
	uint32_t page = 0;
	uint8_t *data;
	uint8_t dataLen = 0;
	HAL_StatusTypeDef res;
	uint8_t offset = 0;
	switch (sector) {
	case ATTENUATION:
		page = M24C64_PAGE_ADDR(0);
		offset = ATTENUATION_OFFSET;
		if (a->val < 0 || a->val > 31)
			a->val = 0;
		data = (uint8_t*) &(a->val);
		dataLen = sizeof(a->val);
		break;
	default:
		return (HAL_ERROR);
		break;
	}

	res = savePage(i2c, page, data, offset, dataLen);
	return (res);
}

void paEnableInit(POWER_AMPLIFIER_t *pa) {
	pa->port = PA_HAB_GPIO_Port;
	pa->pin = PA_HAB_Pin;
	HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_RESET);
}

void paAtteunatorInit(POWER_AMPLIFIER_t *pa) {
	pa->attenuator = malloc(sizeof(BDA4601_t));
	if (pa->attenuator == NULL)
		Error_Handler();

	pa->attenuator->clkPin = CLK_ATT_Pin;
	pa->attenuator->clkPort = CLK_ATT_GPIO_Port;
	pa->attenuator->dataPin = DATA_ATT_Pin;
	pa->attenuator->dataPort = DATA_ATT_GPIO_Port;
	pa->attenuator->lePin = LE_ATT_Pin;
	pa->attenuator->lePort = LE_ATT_GPIO_Port;
	pa->attenuator->val = 0;
	readEepromData(pa, ATTENUATION);
	setInitialAttenuation(pa->attenuator, STARTING_MILLIS);
}

void paAdcInit(POWER_AMPLIFIER_t *pa) {
	pa->adc = adcInit(ADC1);
	if (pa->adc == NULL)
		Error_Handler();

	configADC(pa->adc->reg);
}

void paUsart1Init(POWER_AMPLIFIER_t *pa) {
	pa->serial = uart(USART1);
	if (pa->serial == NULL)
		Error_Handler();

	uartInit(pa->serial->reg);
	pa->serial->dePort = DE_485_GPIO_Port;
	pa->serial->dePin = DE_485_Pin;
}

void paLedInit(POWER_AMPLIFIER_t *pa) {
	pa->led = malloc(LED_CHANNELS * sizeof(LED_INFO_t*));
	if (pa->led == NULL)
		Error_Handler();

	for (int i = 0; i < LED_CHANNELS; i++) {
		pa->led[i] = malloc(sizeof(LED_INFO_t));
		if (pa->led[i] == NULL)
			Error_Handler();
	}
	pa->led[KEEP_ALIVE]->port = KA_GPIO_Port;
	pa->led[KEEP_ALIVE]->pin = KA_Pin;
	pa->led[CURRENT_LOW]->port = CURR_L_GPIO_Port;
	pa->led[CURRENT_LOW]->pin = CURR_L_Pin;
	pa->led[CURRENT_HIGH]->port = CURR_H_GPIO_Port;
	pa->led[CURRENT_HIGH]->pin = CURR_H_Pin;
	pa->led[CURRENT_NORMAL]->port = CURR_N_GPIO_Port;
	pa->led[CURRENT_NORMAL]->pin = CURR_N_Pin;
	pa->led[TEMPERATURE_OK]->port = TEMP_OK_GPIO_Port;
	pa->led[TEMPERATURE_OK]->pin = TEMP_OK_Pin;
	pa->led[TEMPERATURE_HIGH]->port = TEMP_HIGH_GPIO_Port;
	pa->led[TEMPERATURE_HIGH]->pin = TEMP_HIGH_Pin;
	pa->led[KEEP_ALIVE]->startMillis = HAL_GetTick();
	pa->led[CURRENT_LOW]->startMillis = 0;
	pa->led[CURRENT_HIGH]->startMillis = 0;
	pa->led[CURRENT_NORMAL]->startMillis = 0;
	pa->led[TEMPERATURE_OK]->startMillis = 0;
	pa->led[TEMPERATURE_HIGH]->startMillis = 0;
	for (uint8_t i = 0; i < LED_CHANNELS; i++)
		HAL_GPIO_WritePin(pa->led[i]->port, pa->led[i]->pin, GPIO_PIN_SET);
}

void paDacInit(POWER_AMPLIFIER_t *pa) {
	PA_STATUS_t status;
	pa->poutDac = MCP4725_init(pa->i2c, MCP4706_CHIP_ADDR, REFERENCE_VOLTAGE);
	status = setDacLevel(pa, POUT_NORMAL);
	if (status != PA_OK)
		Error_Handler();
}

void serialRestart(POWER_AMPLIFIER_t *pa, uint16_t timeout) {
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
	if (HAL_GetTick() - pa->serial->startTicks > timeout) {
		// Disable UART1
		USART1->CR1 &= ~USART_CR1_UE;
		HAL_Delay(1);
		// Enable UART1
		USART1->CR1 |= USART_CR1_UE;
		memset(pa->serial->data, 0, UART_SIZE);
		pa->serial->len = 0;
		pa->serial->startTicks = HAL_GetTick();
	}
}

void paRawToReal(POWER_AMPLIFIER_t *pa) {
	ADC_t *a = pa->adc;

	pa->pReflected = arduino_map_float(a->ma[PREF_CH], PREF_CH_L, PREF_CH_H,
	PREF_REAL_L, PREF_REAL_H);
	pa->pOut = arduino_map_float(a->ma[POUT_CH], POUT_CH_L, POUT_CH_H,
	POUT_REAL_L, POUT_REAL_H);
	pa->gain = arduino_map_float(a->ma[GAIN_CH], GAIN_CH_L, pa->agcRef,
	GAIN_REAL_L, GAIN_REAL_H);
	pa->pIn = arduino_map_float(a->ma[PIN_CH], PIN_CH_L, PIN_CH_H,
	PIN_REAL_L, PIN_REAL_H);

	pa->vswr = calculate_vswr(pa->pOut, pa->pReflected);

	pa->curr = arduino_map_uint16_t(a->ma[CURRENT_CH], CURR_CH_L, CURR_CH_H,
	CURR_REAL_L, CURR_REAL_H);

	pa->vol = arduino_map_float(a->ma[VOLTAGE_CH], VOLT_CH_L, VOLT_CH_H,
	VOLT_REAL_L, VOLT_REAL_H);
	pa->vol /= 100;

	pa->tempOut = readTemperature();
	pa->temp = lm75Read(pa->i2c);
}

void printParameters(POWER_AMPLIFIER_t *pa) {
	UART_t *u = pa->serial;
	u->len =
			sprintf((char*) u->data,
					"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
					(int8_t) pa->pOut, pa->attenuator->val, (int8_t) pa->gain,
					(int8_t) pa->pIn, (uint16_t) pa->curr, (uint8_t) pa->vol);
	// Send response via UART
	SET_BIT(u->dePort->ODR, u->dePin);
	uartSend(u);
	CLEAR_BIT(u->dePort->ODR, u->dePin);
	memset(u->data, 0, sizeof(u->data));
	u->len = 0;
}

void printRaw(POWER_AMPLIFIER_t *pa) {
	UART_t *u = pa->serial;
	ADC_t *a = pa->adc;
	u->len = sprintf((char*) u->data,
			"Pout %d  \t Gain %u \t Curent %u \t Voltage %u\r\n",
			a->ma[POUT_CH], a->ma[GAIN_CH], a->ma[CURRENT_CH],
			a->ma[VOLTAGE_CH]);
	// Send response via UART
	SET_BIT(u->dePort->ODR, u->dePin);
	uartSend(u);
	CLEAR_BIT(u->dePort->ODR, u->dePin);
	memset(u->data, 0, sizeof(u->data));
	u->len = 0;
}

// Function to find the closest value to 'x' in the lookup table
float find_closest(float x) {
	int index = (int) ((x + 0.05) * 2.0f); // Adding 0.05 to round to the nearest integer
	if (index < 0) {
		index = 0;
	} else if (index >= sizeof(pow_table) / sizeof(float)) {
		index = sizeof(pow_table) / sizeof(float) - 1;
	}
	return (pow_table[index]);
}

float calculate_vswr(float Pf_db, float Pr_db) {
	/* Validate the arguments */
	if (Pf_db < Pr_db) {
		/* Invalid case: reflected power is greater than transmitted power */
		return (0); /* Return a negative value to indicate an error */
	}
	if (fabs(Pf_db - Pr_db) < epsilon) {
		/* Degenerate case: reflected power is equal to transmitted power */
		return (0); /* Return infinity to indicate an infinite VSWR */
	}
	// Calculate the reflection coefficient in decibels
	float gamma_db = fabsf(Pr_db - Pf_db);

	// Find the closest value to the reflection coefficient in the lookup table
	float gamma = find_closest(gamma_db);

	// Calculate the VSWR
	float vswr = (1.0f + gamma) / (1.0f - gamma);

	return (fabsf(vswr));
}

float arduino_map_float(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t arduino_map_float_to_uin16_t(float value, float in_min, float in_max,
		uint16_t out_min, uint16_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t arduino_map_uint16_t(uint16_t value, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int8_t arduino_map_int8(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

PA_STATUS_t setDacLevel(POWER_AMPLIFIER_t *pa, POUT_LEVEL_t level) {
	uint8_t res = 0;
	PA_STATUS_t status;

	if (level >= POUT_LEVEL_VALUES)
		return (PA_ERROR);
	float storedVoltage = MCP4725_getVoltage(pa->poutDac);

	if (fabs(storedVoltage - POUT_REF[level]) > EPSILON)
		res = MCP4725_setVoltage(pa->poutDac, POUT_REF[level],
				MCP4725_EEPROM_MODE, MCP4725_POWER_DOWN_OFF);
	status = (res == 1) ? PA_OK : PA_ERROR;
	if (PA_OK){
		pa->pOutLevel = level;
		pa->agcRef = arduino_map_float_to_uin16_t(POUT_REF[level],0,REFERENCE_VOLTAGE,0,4095);
	}
	return (status);
}

