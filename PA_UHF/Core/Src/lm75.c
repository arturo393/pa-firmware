#include "lm75.h"

void lm75_init(void) {
	uint8_t cmd[2];
	cmd[0] = LM75_Conf;
	cmd[1] = 0x0;
	i2c1_byte_tx(LM75_ADDR << 1, cmd, 2);
}

float lm75_read(void) {
	uint8_t cmd[2];
	float result = 0;
	cmd[0] = LM75_Temp;

	i2c1_byte_tx( LM75_ADDR << 1, cmd, 1); // Send command string
	HAL_Delay(1);
	i2c1MasterFrameRx(LM75_ADDR << 1 | 1, cmd, 2); // Send command string
	result = (float) ((cmd[0] << 8) | cmd[1]) / 256.0f;
	return result;
}

HAL_StatusTypeDef lm75Init(I2C_HandleTypeDef *i2c) {
	uint8_t cmd[2];
	uint8_t len;
	cmd[0] = LM75_Conf;
	cmd[1] = 0x0;
	len = sizeof(cmd);
	i2c1_byte_tx(LM75_ADDR << 1, cmd, 2);
	return (HAL_I2C_Master_Transmit(i2c, LM75_ADDR << 1, cmd, len,
	HAL_MAX_DELAY));
}

uint8_t lm75Read(I2C_HandleTypeDef *i2) {
	uint8_t cmd[2];
	uint8_t len;
	uint8_t result;
	cmd[0] = LM75_Temp;
	len = sizeof(cmd);

	i2c1_byte_tx( LM75_ADDR << 1, cmd, 1); // Send command string
	HAL_Delay(1);
	i2c1MasterFrameRx(LM75_ADDR << 1 | 1, cmd, 2); // Send command string

	result = ((cmd[0] << 8) | cmd[1]);
	return (result / 256);
}
