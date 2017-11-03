#include "../crc16.h"

static inline u16 crc_itu_t(u16 crc, const u8 *buffer, size_t len)
{
	return crc16_block(crc, buffer, len);
}
