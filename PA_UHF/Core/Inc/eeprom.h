/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : eeprom.h
 * @brief          : Header for eeprom.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c1.h"

uint8_t EEPROM_Read(uint8_t address);
void EEPROM_Write(uint8_t address, uint8_t data);
void EEPROM_2byte_Write(uint8_t addr, uint16_t data);
uint16_t EEPROM_2byte_Read(uint8_t address);

void eeprom_write(uint8_t addr, uint16_t data);
uint16_t eeprom_read(uint8_t address);
uint8_t eeprom_1byte_read(uint8_t eaddress);
void eeprom_1byte_write(uint8_t address, uint8_t data);
void eeprom_test_write(uint8_t caddress, uint8_t address, uint8_t data);
uint8_t eeprom_test_read(uint8_t caddress, uint8_t eaddress);

#define EEPROM_CHIP_ADDR 0x50
#define EEPROM_PAGE_SIZE 8
#define EEPROM_PAGE_NUM 32

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

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
