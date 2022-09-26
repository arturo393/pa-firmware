/*
 * led.h
 *
 *  Created on: Sep 26, 2022
 *      Author: sigmadev
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

#define  current_low_led_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD12)
#define current_low_led_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD12)

#define  current_normal_led_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD11)
#define current_normal_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD11)

#define  current_high_led_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD10)
#define current_high_led_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD10)


#define sys_rp_led_off() CLEAR_BIT(GPIOC->ODR,GPIO_ODR_OD6)
#define sys_rp_led_on() SET_BIT(GPIOC->ODR,GPIO_ODR_OD6)

#define  temperature_ok_led_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD9)
#define temperature_ok_led_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD9)

#define temperature_high_led_off() CLEAR_BIT(GPIOA->ODR,GPIO_ODR_OD9)
#define temperature_highled_on() SET_BIT(GPIOA->ODR,GPIO_ODR_OD9)

void led_init(void);



#endif /* INC_LED_H_ */
