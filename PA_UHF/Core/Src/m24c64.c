#include <m24c64.h>



HAL_StatusTypeDef saveByte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,
		uint8_t offset) {
	return (savePage(i2c, page, data, offset, 1));
}

uint8_t readByte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t offset) {
	HAL_StatusTypeDef res;
	uint8_t data = 0;
	res = readPage(i2c, page, &data, offset, 1);
	if (res != HAL_OK)
		return (-1);
	return (data);
}

HAL_StatusTypeDef save2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,
		uint8_t offset) {
	return (savePage(i2c, page, data, offset, sizeof(uint16_t)));
}

HAL_StatusTypeDef save4Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,
		uint8_t offset) {
	return (savePage(i2c, page, data, offset, sizeof(uint32_t)));
}

uint32_t read4Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t offset) {
	HAL_StatusTypeDef res;
	uint32_t data = 0;
	res = readPage(i2c, page, (uint8_t*) &data, offset, 4);
	if (res != HAL_OK)
		return (-1);
	return (data);
}

uint16_t read2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t offset) {
	HAL_StatusTypeDef res;
	uint32_t data = 0;
	res = readPage(i2c, page, (uint8_t*) &data, offset, 2);
	if (res != HAL_OK)
		return (-1);
	return (data);
}


HAL_StatusTypeDef readPage(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,
		uint8_t offset, uint8_t size) {
	uint8_t buff[1] = { 0 };
	uint16_t MemAddress = page << PADDRPOSITION | offset;
	buff[0] = (uint8_t) MemAddress & 0xff;
	HAL_StatusTypeDef res;

	res = HAL_I2C_Master_Transmit(i2c, EEPROM_ADDR, buff, 1, 200);

	if (res != HAL_OK)
		return (res);

	HAL_Delay(5);
	res = HAL_I2C_Master_Receive(i2c, EEPROM_ADDR, data, size, 200);
	if (res != HAL_OK)
		return (res);

	return (res);
}

HAL_StatusTypeDef savePage(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,
		uint8_t offset, uint8_t size) {
	uint8_t buff[16 + 1];
	uint8_t read[16];
	uint8_t i = 0;
	HAL_StatusTypeDef res;
	res = readPage(i2c, page, read, offset, size);
	uint8_t notEqual = 0;

	for (i = 0; i < size; i++)
		if (data[i] != read[i]) {
			notEqual = 1;
			break;
		}

	if (notEqual == 1) {
		buff[0] = (uint8_t) (page << PADDRPOSITION | offset) & 0xff;
		for (i = 0; i < size; i++) {
			buff[i + 1] = data[i];
		}
		res = HAL_I2C_Master_Transmit(i2c, EEPROM_ADDR, buff, 1, 200);
	}
	HAL_Delay(6);

	return (res);
}
