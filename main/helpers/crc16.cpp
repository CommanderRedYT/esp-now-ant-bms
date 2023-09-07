#include "crc16.h"

namespace helpers {
uint16_t crc16(const uint8_t *data, uint8_t data_length)
{
    uint16_t crc = 0xFFFF;
    while (data_length--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if ((crc & 0x01) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
} // namespace helpers
