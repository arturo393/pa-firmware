
/*
 * led.h
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"
#include "stdlib.h"

#define  current_low_led_on() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD12)
#define current_low_led_off() SET_BIT(GPIOA->ODR,GPIO_ODR_OD12)

#define  current_normal_led_on() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD11)
#define current_normal_led_off() SET_BIT(GPIOA->ODR,GPIO_ODR_OD11)

#define  current_high_led_on() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD10)
#define current_high_led_off() SET_BIT(GPIOA->ODR,GPIO_ODR_OD10)

#define sys_rp_led_on() CLEAR_BIT(GPIOC->ODR,GPIO_ODR_OD6)
#define sys_rp_led_off() SET_BIT(GPIOC->ODR,GPIO_ODR_OD6)

#define  temperature_ok_led_on() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD9)
#define temperature_ok_led_off() SET_BIT(GPIOA->ODR,GPIO_ODR_OD9)

#define temperature_high_led_on() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD8)
#define temperature_high_led_off() SET_BIT(GPIOA->ODR,GPIO_ODR_OD8)

#define  LED_KA_STATE_TIMEOUT  1000
#define LED_KA_ON_TIMEOUT  50
#define LED_MAX_CURRENT 600
#define LED_MIN_CURRENT 100
#define LED_MAX_TEMPERATURE 75

typedef enum{
KEEP_ALIVE,
CURRENT_LOW,
CURRENT_NORMAL,
CURRENT_HIGH,
TEMPERATURE_OK,
TEMPERATURE_HIGH,
LED_CHANNELS
}LED_CH_t;

typedef struct{
	uint32_t startMillis;
	GPIO_TypeDef *port;
	uint16_t pin;
}LED_INFO_t;

typedef struct LED{
	uint32_t ka;
	uint32_t cl_counter;
	uint32_t cn_counter;
	uint32_t ch_counter;
	uint32_t sysrp_counter;
	uint32_t tok_counter;
	uint32_t th_counter;
}LED_t;


LED_INFO_t** ledInit(uint8_t ch);
void currentUpdate(LED_INFO_t **l, int16_t current);
void led_init(LED_t *led);
void led_off(void);
void kaUpdate(LED_INFO_t *l);
void led_reset(LED_t *l);
void led_current_update(int16_t current);
void temperatureUpdate(LED_INFO_t **l, uint8_t temperature);


#endif /* INC_LED_H_ */
