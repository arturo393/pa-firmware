#include "lm75.h"

void lm75_init(void ) {
	uint8_t cmd[2];
	cmd[0] = LM75_Conf;
	cmd[1] = 0x0;
	i2c1_byte_tx(LM75_ADDR<<1, cmd, 2);
}

float lm75_read(void) {
	uint8_t cmd[2];
	float result = 0;
	cmd[0] = LM75_Temp;

	i2c1_byte_tx( LM75_ADDR<<1, cmd, 1); // Send command string
	HAL_Delay(1);
	i2c1_buffReceive(LM75_ADDR<<1 | 1, cmd, 2); // Send command string
	result = (float) ((cmd[0] << 8) | cmd[1]) / 256.0f;
	return result;
}
