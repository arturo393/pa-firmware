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

#define CHIP_ADDR 0x50
#define PAGE_SIZE 8
#define PAGE_NUM 32

#define  BASE_ADDR 0x05
#define 	ATT_VALUE_ADDR BASE_ADDR  + 8
#define 	POUT_ADC_MAX_ADDR  BASE_ADDR + 16
#define 	POUT_ADC_MIN_ADDR  BASE_ADDR + 32
#define 	POUT_IS_CALIBRATED_ADDR BASE_ADDR + 40
#define 	PIN_ADC_MAX_ADDR  BASE_ADDR + 48
#define 	PIN_ADC_MIN_ADDR  BASE_ADDR + 56
#define 	PIN_IS_CALIBRATED_ADDR  BASE_ADDR + 64
#define 	VSWR_ADC_MAX_ADDR  BASE_ADDR + 72
#define 	VSWR_ADC_MIN_ADDR  BASE_ADDR + 80
#define 	VSWR_IS_CALIBRATED_ADDR  BASE_ADDR + 88

uint8_t m24c64_Read(uint8_t address);
void m24c64_Write(uint8_t address, uint8_t data);
void m24c64_2byte_Write(uint8_t addr, uint16_t data);
uint16_t m24c64_2byte_Read(uint8_t address);

void m24c64_write(uint8_t addr, uint16_t data);
uint16_t m24c64_read(uint8_t address);
uint8_t m24c64_1byte_read(uint8_t eaddress);
void m24c64_1byte_write(uint8_t address, uint8_t data);
void m24c64_test_write(uint8_t caddress, uint8_t address, uint8_t data);
uint8_t m24c64_test_read(uint8_t caddress, uint8_t eaddress);

typedef struct  m24c64{
uint8_t addrs;
uint8_t *data;
} m24c6403_t;

#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
