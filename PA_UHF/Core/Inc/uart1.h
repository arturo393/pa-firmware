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
#include "string.h"

#define UART_SIZE 100
#define  RX_BUFFLEN 25
#define TX_BUFFLEN  100
#define SECONDS(x) x*1000
#define SYS_FREQ 64000000
#define APB_FREQ SYS_FREQ
#define HS16_CLK 16000000
#define BAUD_RATE 115200

typedef enum{
	READY,
	NOT_READY

}UART_Status;

typedef struct UART1 {
	uint8_t rx_buffer[RX_BUFFLEN];
	uint8_t tx_buffer[RX_BUFFLEN];
	uint8_t rx_count;
	uint32_t timeout;
} UART1_t;

typedef struct {
	uint8_t data[UART_SIZE];
	uint8_t len;
	uint32_t operationTimeout;
	UART_Status status;
	uint32_t startTicks;
	GPIO_TypeDef* dePort;
	uint16_t dePin;
	USART_TypeDef* reg;
} UART_t;

UART_t *uart(USART_TypeDef *uart);
uint8_t uartSend(UART_t *u);

uint8_t  uart1_clean_by_timeout(UART1_t* uart1,const char* str);
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
void uartInit(USART_TypeDef* reg);

#endif /* INC_UART1_H_ */
