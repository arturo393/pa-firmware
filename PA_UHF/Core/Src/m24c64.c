#include <m24c64.h>

//extern I2C_HandleTypeDef hi2c1;

/*
 *
 uint8_t EEPROM_Read(uint8_t address) {
 uint8_t buff[2];
 buff[0] = address;
 HAL_I2C_Master_Transmit(&hi2c1, EEPROM_CHIP_ADDR << 1, buff, 1, 100);
 HAL_I2C_Master_Receive(&hi2c1, EEPROM_CHIP_ADDR << 1 | 1, &buff[1], 1, 100);
 return buff[1];
 }

 void EEPROM_Write(uint8_t address, uint8_t data) {
 uint8_t buff[2];
 uint8_t stored_data;
 buff[0] = address;
 buff[1] = data;

 stored_data = EEPROM_Read(address);

 if (stored_data != data)
 HAL_I2C_Master_Transmit(&hi2c1, EEPROM_CHIP_ADDR << 1, buff, 2, 100);
 }

 void EEPROM_2byte_Write(uint8_t addr, uint16_t data) {
 EEPROM_Write(addr, data & 0xff);
 HAL_Delay(5);
 EEPROM_Write(addr + 1, data >> 8);
 }

 uint16_t EEPROM_2byte_Read(uint8_t address) {
 uint16_t data = 0;
 data = EEPROM_Read(address + 1) << 8;
 HAL_Delay(5);
 data |= EEPROM_Read(address);

 return data;
 }
 */

void m24c64_page_read(uint8_t address, uint8_t page, uint8_t *data) {
	uint8_t buff[2] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION;

	buff[0] = MemAddress >> 8;
	buff[1] = MemAddress & 0xff;

	i2c1_byte_tx(CHIP_ADDR, buff, 2);
	i2c1_buffReceive(CHIP_ADDR, data, 32);
}

void m24c64_read_N(uint8_t page, uint8_t *data, uint8_t offset, uint8_t size) {
	uint8_t buff[2] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION | offset;

	buff[0] = MemAddress >> 8;
	buff[1] = MemAddress & 0xff;

	i2c1_byte_tx(CHIP_ADDR, buff, 2);
	i2c1_buffReceive(CHIP_ADDR, data, size);
}

void m24c64_write_N(uint8_t page, uint8_t *data, uint8_t offset, uint8_t size) {
	uint8_t buff[size + 2];
	uint8_t read[size];

	m24c64_read_N(page, read, offset, size);

//	HAL_Delay(5);
	if (strncmp((const char*) data, (const char*) read, (size_t) size)) {
		buff[0] = (page << PADDRPOSITION | offset) >> 8;
		buff[1] = (page << PADDRPOSITION | offset) & 0xff;
		for (int i = 0; i < size; i++) {
			buff[i + 2] = data[i];
		}
		i2c1_byte_tx(CHIP_ADDR, buff, size + 2);
	}
	HAL_Delay(6);
}

void m24c64_init_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	m24c64_read_N(BASE_ADDR, buff, addr, 1);
	if (!(buff[0] == IS_READY)) {
		buff[0] = value >> 8;
		buff[1] = value & 0xff;
		m24c64_write_N(BASE_ADDR, buff, addr + 1, 2);
	}
}

void m24c64_store_16uvalue(M24C64_ADDR_t addr, uint16_t value) {
	uint8_t buff[2];
	buff[0] = value >> 8;
	buff[1] = value & 0xff;
	m24c64_write_N(BASE_ADDR, buff, addr + 1, 2);
	buff[0] = addr;
	m24c64_write_N(BASE_ADDR, buff, addr, 1);
}

