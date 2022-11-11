/*
 * ds18b20.h
 *
 *  Created on: Nov 7, 2022
 *      Author: sigmadev
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "main.h"
#include "stdbool.h"

#define set_vdd_low() CLEAR_BIT(GPIOB->ODR, GPIO_ODR_OD2)
#define set_vdd_high() SET_BIT(GPIOB->ODR, GPIO_ODR_OD2)

uint8_t DS18B20_Start (void);
void ds18b20_write_byte (uint8_t data);
uint8_t ds18b20_read_byte(void);
void set_vdd_as_output();
void set_vdd_as_input();
void ds18b20_convert();
float ds18b20_read_temperature();
void ds18b20_init();
void delay_us(uint16_t us);


#endif /* INC_DS18B20_H_ */
