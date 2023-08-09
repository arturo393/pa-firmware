/*
 * ltel.h
 *
 *  Created on: 27-09-2022
 *      Author: sigmadev
 */

#ifndef INC_LTEL_H_
#define INC_LTEL_H_

#include "main.h"
#include "uart1.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "adc.h"
#include "MCP4725.h"
#include "rdss.h"
#include "bda4601.h"
#include "m24c64.h"
#include "led.h"
#include "lm75.h"
#include "ds18b20.h"

#define REFERENCE_VOLTAGE 3.3
#define EPSILON 0.0001
#define MAX_TEMPERATURE 75
#define SAFE_TEMPERATURE 50
#define MAX_VSWR 1.7
#define SAFE_VSWR 1.0

#define pa_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD3)
#define pa_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD3)
#define pa_state()  READ_BIT(GPIOA->ODR,GPIO_ODR_OD3) ? 1 : 0

#define ATTENUATION_OFFSET 0
#define BW_OFFSET 1
#define CR_OFFSET 2
#define FUNCTION_OFFSET 3
#define ID_OFFSET 4
#define UL_OFFSET 0
#define DL_OFFSET 4
#define AOUT_0_10V_OFFSET 0
#define AOUT_4_20mA_OFFSET 2
#define AOUT_0_20mA_OFFSET 4
#define DOUT_OFFSET 6

// ad8363
#define POUT_CH_L   800
#define POUT_CH_H   2033
#define POUT_REAL_L 0
#define POUT_REAL_H 19.8

// MAX4003
#define PIN_CH_L   906
#define PIN_CH_H   1310
#define PIN_REAL_L 10
#define PIN_REAL_H 20.0

// MAX4003
#define PREF_CH_L   800
#define PREF_CH_H   2033
#define PREF_REAL_L 0
#define PREF_REAL_H 19.8

// SALIUDA DEL COMPARADOR QUE VA DIRECTO AL ATENUADOR

#define GAIN_CH_L   1283
#define GAIN_CH_H   3209
#define GAIN_REAL_L 0
#define GAIN_REAL_H 25

#define CURR_CH_H   665
#define CURR_REAL_H 383
#define CURR_CH_L   45
#define CURR_REAL_L 25

#define VOLT_CH_L   779
#define VOLT_CH_H   1303
#define VOLT_REAL_L 1239
#define VOLT_REAL_H 2125

#define MAX4003_DBM_MAX 0
#define MAX4003_DBM_MIN -30
#define MAX4003_ADC_MAX ((uint16_t) 1888)
#define MAX4003_ADC_MIN ((uint16_t) 487)

#define MAX4003_IS_CALIBRATED 0xAA


typedef enum {
	POUT_LOW, POUT_MEDIUM, POUT_NORMAL, POUT_HIGH
} POUT_LEVEL_t;

typedef enum {
	ATTENUATION,
	BANDWITH,
	CODING_RATE,
	FUNCTION,
	ID,
	UPLINK,
	DOWNLINK,
	AOUT_0_10V,
	AOUT_4_20mA,
	AOUT_0_20mA,
	DOUT
} EEPROM_SECTOR_t;

typedef enum {
	PA_UPDATE, PA_WAIT, PA_ERROR, PA_OK
} PA_STATUS_t;

typedef enum MODULE_ID {
	ID0 = 0x00, ID8 = 0x08, ID9 = 0X09
} Id_t;

typedef enum MODULE_S {
	OFF, ON
} State_t;

typedef struct MODULE {
	uint8_t att;
	int8_t gain;
	int8_t pOut;
	int8_t pRef;
	float vol;
	int8_t pIn;
	uint16_t curr;
	uint8_t enable;
	float temp;
	float tempOut;
	float vswr;
	POUT_LEVEL_t pOutLevel;
	uint8_t id;
	uint8_t function;
	PA_STATUS_t status;
	GPIO_TypeDef *port;
	uint16_t pin;
	ADC_t *adc;
	MCP4725 *poutDac;
	UART_t *serial;
	BDA4601_t *attenuator;
	I2C_HandleTypeDef *i2c;

	LED_INFO_t **led;
} POWER_AMPLIFIER_t;

extern const uint8_t MODULE_ADDR;
extern const uint8_t MODULE_FUNCTION;
extern const uint8_t MCP4706_CHIP_ADDR;

POWER_AMPLIFIER_t* paInit();
void module_calc_parameters(POWER_AMPLIFIER_t m, uint16_t *media_array);
void startTimer3(uint8_t seconds);
void tooglePa(POWER_AMPLIFIER_t *pa);
void print_parameters(UART1_t *u, POWER_AMPLIFIER_t *pa);
void printParameters(POWER_AMPLIFIER_t *pa);
void printRaw(POWER_AMPLIFIER_t *pa);
float vswrCalc(float pf_db, float pr_db);
float calculate_vswr(float pt_dbm, float pr_dbm);
float arduino_map_float(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max);
uint16_t arduino_map_uint16_t(uint16_t value, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max);
int8_t arduino_map_int8_t(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max);
void rs485_set_query_frame(POWER_AMPLIFIER_t *module);
void processReceivedSerial(POWER_AMPLIFIER_t *p);
uint8_t exec(POWER_AMPLIFIER_t *pa, uint8_t *dataReceived);
uint8_t readEepromData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector);
HAL_StatusTypeDef saveData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector);
void paEnableInit(POWER_AMPLIFIER_t *pa);
void paAtteunatorInit(POWER_AMPLIFIER_t *pa);
void paAdcInit(POWER_AMPLIFIER_t *pa);
void paUsart1Init(POWER_AMPLIFIER_t *pa);
void paLedInit(POWER_AMPLIFIER_t *pa);
void paDacInit(POWER_AMPLIFIER_t *pa);
void serialRestart(POWER_AMPLIFIER_t *pa, uint16_t timeout);
void paRawToReal(POWER_AMPLIFIER_t *pa);
PA_STATUS_t setDacLevel(POWER_AMPLIFIER_t *pa, POUT_LEVEL_t level);
int16_t findNearestValue(int16_t target, const int16_t *adc,
		const int16_t *real, uint8_t samples);
#endif /* INC_LTEL_H_ */
