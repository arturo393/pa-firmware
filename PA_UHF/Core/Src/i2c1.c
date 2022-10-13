/*
 * i2c.c
 *
 *  Created on: Aug 16, 2022
 *      Author: sigmadev
 */

#include "i2c1.h"

void i2c1_init() {

	/* SCL PB8  as alternate */
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE8_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE8_1);
	/* SDC PB9 as alternate */
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE9_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE9_1);
	/* SCL PB8 as open-drain */
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT8);
	/* SDC PB9 as open-drain */
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT9);
	/* SCL PB8 High Speed output */
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED8_0);
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED8_1);
	/* SDC PB9  High Speed output */
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED9_0);
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED9_1);
	/* SCL PB8 as pull-up */
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD8_0);
	SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD8_1);
	/* SDC PB9 as pull-up */
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD9_0);
	SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD9_1);
	/*  PB8 as i2c SCL */
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL8_0);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL8_1);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL8_2);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL8_3);
	/*  PB9 as i2c SDL */
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL9_0);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL9_1);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL9_2);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFSEL9_3);

	/* select normal speed */
	SET_BIT(RCC->APBENR1, RCC_APBENR1_I2C1EN);

	/* i2c disable */
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);

	// TODO revisar porque funciona
	//	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_PRESC, 1);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLDEL, 0x7);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SDADEL, 0x0);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLH, 0x7D);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLL, 0xBC);

	MODIFY_REG(I2C1->TIMINGR, 0X10111111U, 0X10707DBCU);

	/*i2c Rx interrupt enable */
	SET_BIT(I2C1->CR1, I2C_CR1_RXIE);
	SET_BIT(I2C1->CR1, I2C_CR1_TXIE);

	/* i2c enable */
	SET_BIT(I2C1->CR1, I2C_CR1_PE);
}

char i2c1_byteReceive(char saddr, uint8_t N) {
	uint32_t counter = HAL_GetTick();
	bool timeout = false;
	i2c1_start(saddr, READ, N);

	char data = 0;
	for (int i = 0; i < N; i++) {
		while (!READ_BIT(I2C1->ISR, I2C_ISR_RXNE) & !timeout) {
			if (HAL_GetTick() - counter > 500)
				return 0x00;
		}
		data = READ_REG(I2C1->RXDR);

	}
	while (!(READ_BIT(I2C1->ISR, I2C_ISR_STOPF))) {
	}
	SET_BIT(I2C1->ICR, I2C_ICR_STOPCF);

	return data;
}

void  i2c1_buffReceive(char saddr, uint8_t *rcv,  uint8_t N) {
	uint32_t counter = HAL_GetTick();
	bool timeout = false;
	i2c1_start(saddr, READ, N);

	char data = 0;
	for (int i = 0; i < N; i++) {
		while (!READ_BIT(I2C1->ISR, I2C_ISR_RXNE) & !timeout) {
			if (HAL_GetTick() - counter > 500)
				return 0x00;
		}
		rcv[i] = READ_REG(I2C1->RXDR);

	}
	while (!(READ_BIT(I2C1->ISR, I2C_ISR_STOPF))) {
	}
	SET_BIT(I2C1->ICR, I2C_ICR_STOPCF);

}

void i2c1_byteTransmit(char saddr, uint8_t *data, uint8_t N) {
	i2c1_start(saddr, WRITE, N);
	uint32_t counter = HAL_GetTick();

	for (int i = 0; i < N; i++) {
		while (!READ_BIT(I2C1->ISR, I2C_ISR_TXIS)) {

			if (HAL_GetTick() - counter > 500)
				return;
		}
		MODIFY_REG(I2C1->TXDR, I2C_TXDR_TXDATA, data[i]);
	}

	while (!READ_BIT(I2C1->ISR, I2C_ISR_STOPF)) {
	}
	SET_BIT(I2C1->ISR, I2C_ICR_STOPCF);

}

void i2c1_start(char saddr, uint8_t transfer_request, uint8_t N) {
	/*master 7 bit addressing mode */
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ADD10);
	/* set Slave address */
	MODIFY_REG(I2C1->CR2, I2C_CR2_SADD, saddr << I2C_CR2_SADD_Pos);
	/* read 1 byte */
	MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES, N << I2C_CR2_NBYTES_Pos);
	/* stops when NBytes are transferred */
	SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND);
	/* set START condition  automatically changes to master */

	if (transfer_request == 1) {
		/* request a read transfer */
		SET_BIT(I2C1->CR2, I2C_CR2_RD_WRN);
	} else if (transfer_request == 0) {
		/* request a write transfer */
		CLEAR_BIT(I2C1->CR2, I2C_CR2_RD_WRN);
	}

	SET_BIT(I2C1->CR2, I2C_CR2_START);

}

void i2c1_scanner(uint8_t *addr) {
	uint32_t counter = HAL_GetTick();
	uint8_t j = 0;
	bool timeout = false;

	for (int i = 1; i < 128; i++) {
		i2c1_start(i << 1 | 1, READ, 1);
		timeout = false;

		while (!READ_BIT(I2C1->ISR, I2C_ISR_RXNE) & !timeout) {

			if (HAL_GetTick() - counter > 500) {
				counter = HAL_GetTick();
				timeout = true;
			}
		}
		if (!timeout) {
			addr[j++] = i;
			READ_REG(I2C1->RXDR);
		}
	}
}
