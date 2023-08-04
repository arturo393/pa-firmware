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
		p->pIn = 0;
		p->pOut = 0;
		p->temperature = 0;
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
		if (pa->temperatureOut > MAX_TEMPERATURE)
			HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_RESET);
		if (pa->temperatureOut < SAFE_TEMPERATURE) {
			HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_SET);
			if (pa->vswr > MAX_VSWR)
				HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_RESET);
			if (pa->vswr < MAX_VSWR)
				HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_SET);
		}
	}
	if (pa->enable == OFF)
		HAL_GPIO_WritePin(pa->port, pa->pin, GPIO_PIN_RESET);
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
	switch (response[CMD_INDEX]) {
	case QUERY_PARAMETER_LTEL:
		dataLen = sizeof(pa->enable) + sizeof(blank) + sizeof(pa->temperature)
				+ sizeof(pa->gain) + sizeof(pa->vswr) + sizeof(pa->att)
				+ sizeof(pa->pOut) + sizeof(pa->pIn);
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
		memcpy(responsePtr, &(pa->pOut), sizeof(pa->pOut));
		responsePtr += sizeof(pa->pOut);
		memcpy(responsePtr, &(pa->att), sizeof(pa->att));
		responsePtr += sizeof(pa->att);
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
		u->len = sprintf((char*) u->data, "Attenuation %u\r\n",
				pa->attenuator->val);
		bda4601_set_att(pa->attenuator->val, 3);
		// Send response via UART
		SET_BIT(u->dePort->ODR, u->dePin);
		uartSend(u);
		CLEAR_BIT(u->dePort->ODR, u->dePin);
		memset(u->data, 0, sizeof(u->data));
		u->len = 0;
		saveData(pa, ATTENUATION);
		break;
	case SET_ENABLE_PA:
		pa->enable = dataReceived[DATA_LENGHT2_INDEX];
		break;
	case SET_POUTLEVEL:
		pa->pOutLevel = dataReceived[DATA_START_INDEX];
		pa->status = setDacLevel(pa, pa->pOutLevel);
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

	switch (sector) {
	case ATTENUATION:
		attenuator->val = readByte(i2c, M24C64_PAGE_ADDR(0),
		ATTENUATION_OFFSET);
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
	pa->temperatureOut = readTemperature();
	pa->temperature = lm75Read(pa->i2c);
	pa->pRef = arduino_map_float(a->ma[PREF_CH], MAX4003_ADC_MIN,
	MAX4003_ADC_MAX, -30.0, 0.0);
	pa->pOut = arduino_map_float(a->ma[POUT_CH], MAX4003_ADC_MIN,
	MAX4003_ADC_MAX, -30.0, -10.0);

	pa->gain = arduino_map_float(a->ma[PIN_CH], MAX4003_ADC_MIN,
	MAX4003_ADC_MAX, 0.0, 30.0);
	pa->vswr = vswrCalc(pa->pOut, pa->pRef);
	pa->pIn = arduino_map_float(a->ma[PIN_CH], MAX4003_ADC_MIN,
	MAX4003_ADC_MAX, -30.0, 0.0);

	pa->current = arduino_map_float(a->ma[PIN_CH], 705, 626, 400, 460);
	pa->voltage = arduino_map_float(a->ma[VOLTAGE_CH], 779, 1303, 12.39, 21.25);
}

void printParameters(POWER_AMPLIFIER_t *pa) {
	UART_t *u = pa->serial;
	u->len =
			sprintf((char*) u->data,
					"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
					pa->pOut, pa->attenuator->val, pa->gain, pa->pIn,
					pa->current, (uint8_t) pa->voltage);
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

const float pow_table[] = { 1.000000f, 1.122018f, 1.258925f, 1.412538f,
		1.584893f, 1.778279f, 1.995262f, 2.238721f, 2.511886f, 2.818383f,
		3.162278f, 3.548134f, 3.981072f, 4.466836f, 5.011872f, 5.623413f,
		6.309573f, 7.079458f, 7.943282f, 8.912509f, 10.000000f };

// Function to find the closest value to 'x' in the lookup table
float find_closest(float x) {
	int index = (int) (x * 2.0f);
	return (pow_table[index]);
}

// Function to calculate VSWR from reflected power (Pf_db) and output power (Pr_db)
float vswrCalc(float Pf_db, float Pr_db) {
	// Calculate the reflection coefficient in decibels
	float gamma_db = Pr_db - Pf_db;

	// Find the closest value to the reflection coefficient in the lookup table
	float gamma = find_closest(gamma_db);

	// Calculate the VSWR
	float vswr = (1.0f + gamma) / (1.0f - gamma);

	return (vswr);
}
float arduino_map_float(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int8_t arduino_map_int8(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

PA_STATUS_t setDacLevel(POWER_AMPLIFIER_t *pa, POUT_LEVEL_t level) {
	uint8_t res = 0;
	PA_STATUS_t status;
	float newVoltage;

	switch (level) {
	case (POUT_LOW):
		newVoltage = 1.84;
		break;
	case (POUT_MEDIUM):
		newVoltage = 2.01;
		break;
	case (POUT_HIGH):
		newVoltage = 2.35;
		break;
	default:
		newVoltage = 2.01;
		break;
	}
	float storedVoltage = MCP4725_getVoltage(pa->poutDac);

	if (fabs(storedVoltage - newVoltage) > EPSILON)
		res = MCP4725_setVoltage(pa->poutDac, newVoltage, MCP4725_EEPROM_MODE,
				MCP4725_POWER_DOWN_OFF);
	status = (res == 1) ? PA_OK : PA_ERROR;
	return (status);
}
