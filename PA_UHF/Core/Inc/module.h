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

#define MAX4003_DBM_MAX 0
#define MAX4003_DBM_MIN -30
#define MAX4003_ADC_MAX ((uint16_t) 1888)
#define MAX4003_ADC_MIN ((uint16_t) 487)

#define MAX4003_IS_CALIBRATED 0xAA

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
	PA_UPDATE, PA_WAIT

} Status_t;

typedef enum MODULE_ID {
	ID0 = 0x00, ID8 = 0x08, ID9 = 0X09
} Id_t;

typedef enum MODULE_S {
	OFF, ON
} State_t;

typedef struct MODULE {
	uint8_t att;
	uint8_t gain;
	int8_t pOut;
	int8_t pRef;
	uint8_t voltage;
	int8_t pIn;
	uint16_t current;
	uint8_t enable;
	uint8_t temperature;
	uint8_t temperatureOut;
	uint8_t vswr;
	uint8_t id;
	uint8_t function;
	Status_t status;
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

extern const float ADC_CURRENT_FACTOR;
extern const float ADC_VOLTAGE_FACTOR;
extern const uint8_t MCP4706_CHIP_ADDR;
extern const float REFERENCE_VOLTAGE;

POWER_AMPLIFIER_t* paInit();
void module_calc_parameters(POWER_AMPLIFIER_t m, uint16_t *media_array);
void startTimer3(uint8_t seconds);
void tooglePa(POWER_AMPLIFIER_t *pa);
void print_parameters(UART1_t *u, POWER_AMPLIFIER_t *pa);
void printParameters(POWER_AMPLIFIER_t *pa);
void printRaw(POWER_AMPLIFIER_t *pa);
float vswr_calc(int8_t pf, int8_t pr);
float arduino_map_float(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max);
int8_t arduino_map_int8(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max);
void rs485_set_query_frame(POWER_AMPLIFIER_t *module);
void processReceivedSerial(POWER_AMPLIFIER_t *p);
uint8_t exec(POWER_AMPLIFIER_t *pa, uint8_t *dataReceived);
uint8_t readEepromData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector);
HAL_StatusTypeDef saveData(POWER_AMPLIFIER_t *p, EEPROM_SECTOR_t sector);
void paEnableInit(POWER_AMPLIFIER_t *pa);
void paAdcInit(POWER_AMPLIFIER_t *pa);
void paUsart1Init(POWER_AMPLIFIER_t *pa);
void paLedInit(POWER_AMPLIFIER_t *pa);
void serialRestart(POWER_AMPLIFIER_t *pa, uint16_t timeout);
void paRawToReal(POWER_AMPLIFIER_t *pa);
#endif /* INC_LTEL_H_ */
