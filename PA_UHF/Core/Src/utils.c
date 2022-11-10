#include "utils.h"


bool crc_check(uint8_t *frame, uint8_t len, uint8_t *crc_frame) {
	uint16_t crc;
	uint8_t testframe[2];
	crc = crc_get(frame, len);
	memcpy(testframe, &crc, 2);
	if (testframe[1] == crc_frame[1] && testframe[0] == crc_frame[0]) {
		return true;
	}
	return false;
}

uint8_t Crc8(const void* vptr, int len) {
  const uint8_t *data = vptr;
  unsigned crc = 0;
  int i, j;
  for (j = len; j; j--, data++) {
    crc ^= (*data << 8);
    for(i = 8; i; i--) {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}
