/*
 * uart1.c
 *
 *  Created on: Aug 29, 2022
 *      Author: sigmadev
 */

#include <uart1.h>

void uart1_gpio_init() {
	/**USART1 GPIO Configuration
	 PA9     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */
	/* PA9 TX as alter */
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE9_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE9_1);
	/* PA9 TX as alter */
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE10_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE10_1);
	/* PA9 Tx open drain */
	CLEAR_BIT(GPIOA->MODER, GPIO_OTYPER_OT9);
	/* PA9 Tx open drain */
	CLEAR_BIT(GPIOA->MODER, GPIO_OTYPER_OT10);
	/* PA9 Tx pull up */
	CLEAR_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD9_0);
	SET_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD9_1);
	/* PA10 RX pull up */
	CLEAR_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD10_0);
	SET_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD10_1);
	/*  PA9 Tx low speed */
	CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED9_0);
	CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED9_1);
	/* PA10 RX low speed */
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED10_0);
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED10_1);
	/*PA9 TX   AF1 as alter   */
	SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_0);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_1);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_2);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_3);
	/* PA10 RX  AF1 as alter   */
	SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_0);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_1);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_2);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_3);
}

void uart1_init(uint32_t pclk, uint32_t baud_rate, UART1_t *u) {
	uint32_t br_value = 0;

	uart1_gpio_init();

	/*enable clock access to USART1 */
	SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);
	if (pclk == 16000000) {
		/*set HSI 16 CLK */
		CLEAR_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL_0);
		SET_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL_1);
	}
	//MODIFY_REG(USART1->PRESC,USART_PRESC_PRESCALER,0x0010);
	/* set baud rate */
	br_value = (pclk) / baud_rate;
	USART1->BRR = (uint16_t) br_value;
	/* transmitter enable*/
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE;

	u->rx_buffer = (uint8_t*) malloc(RX_BUFFLEN);
	u->tx_buffer = (uint8_t*) malloc(TX_BUFFLEN);
	u->is_data_ready = false;
	u->rx_count = 0;

	uart1_clean_buffer(u);

	/* enable FIFO */
	SET_BIT(USART1->CR2, USART_CR1_FIFOEN);
	/* enable Rx timeout */
	//SET_BIT(USART1->CR2, USART_CR2_RTOEN);
	/**/
	//MODIFY_REG(USART1->RTOR,USART_RTOR_RTO,100);
	/*set length */
	/* Enable interrupt */
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE_RXFNEIE);
	NVIC_EnableIRQ(USART1_IRQn);
	//uart1_dma_init();
//
	SET_BIT(USART1->CR1, USART_CR1_UE);
}

void uart1_dma_init() {
	/* enable clk access */
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
	/* clear all interrupt flags on stream */

	/* set the destination buffer */
	/*set the source buffer */

}

void uart1_write(char ch) {
	SET_BIT(GPIOA->ODR, GPIO_ODR_OD15);

	while (!READ_BIT(USART1->ISR, USART_ISR_TXE_TXFNF))
		;
	USART1->TDR = (uint8_t) (ch & 0xFFU);

	while (!READ_BIT(USART1->ISR, USART_ISR_TC))
		;

	CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD15);
}

void uart1_read(char *data, uint8_t size) {
	bool override = READ_BIT(USART1->ISR, USART_ISR_ORE);
	bool data_present = READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE);
//	bool busy = READ_BIT(USART1->ISR, USART_ISR_BUSY);
	bool timeout = false;
	uint16_t rxfne = 0;
	if (size > 1) {
		for (int i = 0; (i < size - 1); i++) {
			while ((!data_present && !override && !timeout)) {
				if (rxfne > 1000)
					timeout = true;
				else
					rxfne++;
			}
			data[i] = USART1->RDR;
			if (override)
				SET_BIT(USART1->ICR, USART_ICR_ORECF);
		}
		rxfne = 0;
	} else {
		while (!READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE)) {
		}
		data[0] = USART1->RDR;
	}
}

uint8_t uart1_1byte_read(void) {
	bool override = READ_BIT(USART1->ISR, USART_ISR_ORE);
	bool data_present = READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE);
//	bool busy = READ_BIT(USART1->ISR, USART_ISR_BUSY);
	if ((data_present || override)) {
		if (override)
			SET_BIT(USART1->ICR, USART_ICR_ORECF);
		return USART1->RDR;
	} else
		return '\0';
}

void  uart1_read_to_frame(UART1_t *u) {
	if (u->rx_count >= RX_BUFFLEN) {
		uart1_clean_buffer(u);
		u->rx_count = 0;
	}
	u->rx_buffer[u->rx_count++] = uart1_1byte_read();
}

void uart1_send_str(char *str) {
	uint8_t i;
	for (i = 0; str[i] != '\0'; i++)
		uart1_write(str[i]);
}

void uart1_send_frame(char *str, uint8_t len) {

	if (len > 0) {
		for (int i = 0; i < len; i++)
			uart1_write(str[i]);
	}
}

void uart1_clean_buffer(UART1_t *u) {
	u->rx_count = 0;
	if (TX_BUFFLEN > RX_BUFFLEN) {
		for (int i = 0; i < TX_BUFFLEN; i++) {
			if (i < RX_BUFFLEN)
				u->rx_buffer[i] = 0x00;
			u->tx_buffer[i] = 0x00;
		}
	} else {
		for (int i = 0; i < RX_BUFFLEN; i++) {
			if (i < TX_BUFFLEN)
				u->tx_buffer[i] = 0x00;
			u->rx_buffer[i] = 0x00;
		}
	}
}

uint8_t uart1_nonblocking_read(void) {
	uint8_t rcvcount = 0;
	uint32_t tickstart = HAL_GetTick();
	bool timeout = false;
	uint8_t timeout_value = 10;

	while (rcvcount < RX_BUFFLEN && timeout == false) {
		if (((HAL_GetTick() - tickstart) > timeout_value)
				|| (timeout_value == 0U)) {
			timeout = true;
			if (READ_BIT(USART1->ISR, USART_ISR_ORE))
				SET_BIT(USART1->ICR, USART_ICR_ORECF);
			if (READ_BIT(USART1->ISR, USART_ISR_IDLE))
				SET_BIT(USART1->ICR, USART_ICR_IDLECF);
			if (READ_BIT(USART1->ISR, USART_ISR_FE))
				SET_BIT(USART1->ICR, USART_ICR_FECF);
		}

		if (READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE))
			return USART1->RDR;
	}
}
