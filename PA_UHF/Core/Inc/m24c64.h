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
#include "string.h"

#define CHIP_ADDR 0xa0
#define PAGE_SIZE 32
#define PAGE_NUM 256
#define IS_READY 0xaa
#define PADDRPOSITION 5
#define MAX_DATA

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xa0

#define BASE_ADDR 0x03

#define M24C64_PAGES 256
#define M24C64_PAGE_SIZE 32

#define M24C64_PAGE_ADDR(page) ((page) * M24C64_PAGE_SIZE)
#define M24C64_PAGE_START_ADDR(page) M24C64_PAGE_ADDR(page)
#define M24C64_PAGE_END_ADDR(page) (M24C64_PAGE_ADDR(page) + M24C64_PAGE_SIZE - 1)

#define M24C64_PAGE0 0
#define M24C64_PAGE1 1
#define M24C64_PAGE2 2

/*
typedef enum{
 	ATT_VALUE_ADDR = BASE_ADDR ,
	POUT_MAX_READY_ADDR,
 	POUT_ADC_MAX_ADDR_0 ,
	POUT_ADC_MAX_ADDR_1,
	POUT_MIN_READY_ADDR,
	POUT_ADC_MIN_ADDR_0,
	POUT_ADC_MIN_ADDR_1,
 	PIN_MAX_READY_ADDR,
 	PIN_ADC_MAX_ADDR_0 ,
 	PIN_ADC_MAX_ADDR_1,
	PIN_MIN_READY_ADDR,
 	PIN_ADC_MIN_ADDR_0 ,
 	PIN_ADC_MIN_ADDR_1,
	VSWR_MAX_READY_ADDR,
	VSWR_ADC_MAX_ADDR_0 ,
 	VSWR_ADC_MAX_ADDR_1,
	VSWR_MIN_READY_ADDR,
 	VSWR_ADC_MIN_ADDR_0,
 	VSWR_ADC_MIN_ADDR_1
}M24C64_ADDR_t;
*/

uint8_t readByte(I2C_HandleTypeDef*i2c ,uint8_t page,uint8_t offset);
uint32_t read4Byte(I2C_HandleTypeDef*i2c ,uint8_t page,uint8_t offset);
uint16_t read2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t offset);
HAL_StatusTypeDef readPage(I2C_HandleTypeDef*i2c ,uint8_t page, uint8_t *data, uint8_t offset, uint8_t size);
HAL_StatusTypeDef savePage(I2C_HandleTypeDef*i2c , uint8_t page, uint8_t *data, uint8_t offset, uint8_t size);
HAL_StatusTypeDef saveByte(I2C_HandleTypeDef *i2c, uint8_t page,uint8_t *data, uint8_t offset);
HAL_StatusTypeDef save2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,uint8_t offset);
HAL_StatusTypeDef save4Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,uint8_t offset);


#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
