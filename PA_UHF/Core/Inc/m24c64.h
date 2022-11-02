/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : m24c64.h
 * @brief          : Header for m24c64.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M24C64_H
#define M24C64_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "i2c1.h"

#define CHIP_ADDR 0xa0
#define PAGE_SIZE 32
#define PAGE_NUM 256


// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xA0

#define BASE_ADDR 0x03


typedef enum{
 	ATT_VALUE_ADDR = BASE_ADDR ,
 	POUT_ADC_MAX_ADDR_0 ,
	POUT_ADC_MAX_ADDR_1,
	POUT_ADC_MIN_ADDR_0,
	POUT_ADC_MIN_ADDR_1,
 	PIN_ADC_MAX_ADDR_0 ,
 	PIN_ADC_MAX_ADDR_1,
 	PIN_ADC_MIN_ADDR_0 ,
 	PIN_ADC_MIN_ADDR_1,
 	VSWR_ADC_MAX_ADDR_0 ,
 	VSWR_ADC_MAX_ADDR_1,
 	VSWR_ADC_MIN_ADDR_0,
 	VSWR_ADC_MIN_ADDR_1 ,
 	POUT_IS_CALIBRATED_ADDR,
 	PIN_IS_CALIBRATED_ADDR,
 	VSWR_IS_CALIBRATED_ADDR,
}M24C64_ADDR_t;


typedef struct M24C64 {
	M24C64_ADDR_t  addrs;
	 uint8_t data[PAGESIZE];
} M24C64_t;

void m24c64_page_read(uint8_t address,uint8_t page, uint8_t *data);
void m24c64_read_N(uint8_t page, uint8_t *data, uint8_t offset,uint8_t size);
void m24c64_write_N(uint8_t page, uint8_t *data, uint8_t offset,uint8_t size);



#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
