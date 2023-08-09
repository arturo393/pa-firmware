#include "ds18b20.h"

void ds18b20_init() {
	/*enable clock access to timer 2 */
	SET_BIT(RCC->APBENR2, RCC_APBENR2_TIM16EN);
	/*set preescaler value */
	TIM16->PSC = 64 - 1; // 64 000 000 / 64  = 1 000 000
	/* set auto-reload */
	TIM16->ARR = 0XFFFF - 1; // 1 000  000 /
	SET_BIT(TIM16->CR1, TIM_CR1_ARPE);
	/* clear counter */
	TIM16->CNT = 0;
	/*enable timer 3*/
	SET_BIT(TIM16->CR1, TIM_CR1_CEN);
	SET_BIT(TIM16->DIER, TIM_DIER_UIE);
//	NVIC_EnableIRQ(TIM16_IRQn);
//	CLEAR_BIT(TIM16->SR, TIM_SR_UIF);
}

void delay_us(uint16_t us) {
	TIM16->CNT = 0;
	TIM16->ARR = us;
	while (TIM16->CNT < us)
		;

}

void pull_down(uint16_t us) {
	set_vdd_as_output();
	set_vdd_low();
	delay_us(us);
	set_vdd_as_input();
}

uint8_t DS18B20_Start(void) {
	uint8_t Response = 0;
	pull_down(480);
	delay_us(80);
	Response = READ_BIT(GPIOB->IDR, GPIO_ODR_OD2) ? 1 : 0;
	delay_us(400); // 480 us delay totally.
	return Response;
}

void ds18b20_write1() {
	set_vdd_as_output();
	set_vdd_low();
	delay_us(10); // wait for 1 us
	set_vdd_as_input();
	delay_us(55); // wait for 60 us
	set_vdd_as_input(); // NOTE review if this is necessary
}

void ds18b20_write0() {
	set_vdd_as_output();
	set_vdd_low();
	delay_us(65); // wait for 60 us
	set_vdd_as_input();
	delay_us(5); // wait for 60 us
	set_vdd_as_input(); // NOTE review if this is necessary
}

void ds18b20_write_byte(uint8_t data) {
	for (int i = 0; i < 8; i++) {
		if ((data & (1 << i)) != 0)
			ds18b20_write1();
		else
			ds18b20_write0();
	}
}

uint8_t ds18b20_read_byte(void) {
	uint8_t value = 0;

	set_vdd_as_input();
	for (int i = 0; i < 8; i++) {
		set_vdd_as_output();
		set_vdd_low();
		delay_us(3);  // wait for > 1us
		set_vdd_as_input();
		delay_us(3);
		if (READ_BIT(GPIOB->IDR, GPIO_ODR_OD2))  // if the pin is HIGH
			value |= 1 << i;  // read = 1
		delay_us(50);  // wait for 60 us
	}
	return value;
}

void set_vdd_as_output() {
	/* PB2  as output */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE2_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE2_1);
}

void set_vdd_as_input() {
	/* PB2  as INPUT */
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE2_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE2_1);
}

void ds18b20_convert() {
	if (!DS18B20_Start()) {
		HAL_Delay(1);
		ds18b20_write_byte(DS18B202_SKIP_ROM); // skip ROM
		ds18b20_write_byte(DS18B20_CONVERT); // convert t
//		HAL_Delay(800);
	}
}

float ds18b20_read_temperature() {
	if (!DS18B20_Start()) {
		uint8_t Temp_byte1;
		uint8_t Temp_byte2;
		int TEMP = 0;
		HAL_Delay(1);
		ds18b20_write_byte(DS18B202_SKIP_ROM); // skip ROM
		ds18b20_write_byte(DS18B20_READ_SCRATCHPAD); // Read Scratch-pad
		TEMP = 0;
		Temp_byte1 = ds18b20_read_byte();
		Temp_byte2 = ds18b20_read_byte();
		TEMP = (Temp_byte2 << 8) | Temp_byte1;
		return (float) TEMP / 16.0;
	}
	return (0.0f);
}

uint8_t readTemperature() {
	if (!DS18B20_Start()) {
		uint8_t Temp_byte1;
		uint8_t Temp_byte2;
		int TEMP = 0;
		HAL_Delay(1);
		ds18b20_write_byte(DS18B202_SKIP_ROM); // skip ROM
		ds18b20_write_byte(DS18B20_READ_SCRATCHPAD); // Read Scratch-pad
		TEMP = 0;
		Temp_byte1 = ds18b20_read_byte();
		Temp_byte2 = ds18b20_read_byte();
		TEMP = (Temp_byte2 << 8) | Temp_byte1;
		return ((uint8_t) (TEMP / 16.0));
	}
	return (0);
}
