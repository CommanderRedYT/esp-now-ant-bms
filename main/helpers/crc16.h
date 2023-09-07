#pragma once

// system includes
#include <cstdint>

namespace helpers {
uint16_t crc16(const uint8_t *data, uint8_t data_length);
} // namespace helpers
