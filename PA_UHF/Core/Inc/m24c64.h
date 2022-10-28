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

#define CHIP_ADDR 0x4f
#define PAGE_SIZE 32
#define PAGE_NUM 256

#define BASE_ADDR 5


typedef enum{
 	ATT_VALUE_ADDR =  1 << BASE_ADDR ,
 	POUT_ADC_MAX_ADDR_0  ,
	POUT_ADC_MAX_ADDR_1,
	POUT_ADC_MIN_ADDR_0,
	POUT_ADC_MIN_ADDR_1,
 	POUT_IS_CALIBRATED_ADDR,
 	PIN_ADC_MAX_ADDR_0,
 	PIN_ADC_MAX_ADDR_1,
 	PIN_ADC_MIN_ADDR_0,
 	PIN_ADC_MIN_ADDR_1,
 	PIN_IS_CALIBRATED_ADDR,
 	VSWR_ADC_MAX_ADDR_0 ,
 	VSWR_ADC_MAX_ADDR_1,
 	VSWR_ADC_MIN_ADDR_0 ,
 	VSWR_ADC_MIN_ADDR_1 ,
 	VSWR_IS_CALIBRATED_ADDR ,
}M24C64_ADDR_t;

typedef struct M24C64 {
	M24C64_ADDR_t  addrs;
	volatile uint8_t data[PAGESIZE];
} M24C64_t;

uint8_t m24c64_Read(uint8_t address);
void m24c64_Write(uint8_t address, uint8_t data);
void m24c64_2byte_Write(uint8_t addr, uint16_t data);
uint16_t m24c64_2byte_Read(uint8_t address);
void m24c64_write(uint8_t addr, uint16_t data);
uint16_t m24c64_read(uint8_t address);
void m24c64_1byte_write(uint8_t address, uint8_t data);
void m24c64_test_write(uint8_t caddress, uint8_t address, uint8_t data);
uint8_t m24c64_test_read(uint8_t caddress, uint8_t eaddress);


void store_2byte(M24C64_ADDR_t addr,uint16_t data );
uint16_t  read_2byte(M24C64_ADDR_t addr);
uint8_t  read_byte(M24C64_ADDR_t addr);
void store_byte(M24C64_ADDR_t addr,uint8_t data );

#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
