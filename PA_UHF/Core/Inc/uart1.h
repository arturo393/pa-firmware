/*
 * uart1.h
 *
 *  Created on: Aug 29, 2022
 *      Author: sigmadev
 */

#ifndef INC_UART1_H_
#define INC_UART1_H_

#include "stm32g0xx_hal.h"
#include "stdbool.h"
#include "stdlib.h"

#define  RX_BUFFLEN 25
#define TX_BUFFLEN  100

typedef struct UART1 {
	uint8_t *rx_buffer;
	uint8_t *tx_buffer;
	uint8_t rx_count;
	bool is_data_ready;
} UART1_t;

void uart1_init(uint32_t, uint32_t, UART1_t*);
void uart1_write(volatile char);
void uart1_send_str( char*);
void uart1_send_frame(char*, uint8_t);
void uart1_read(char*, uint8_t);
void uart1_read_test(char*, uint8_t);
uint8_t uart1_1byte_read(void);
void uart1_read_to_frame(UART1_t *u);
void uart1_clean_buffer(UART1_t*);
uint8_t uart1_nonblocking_read(void);

#endif /* INC_UART1_H_ */
