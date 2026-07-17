#include "algorithm_crc.h"

#include <stddef.h>

uint16_t Algorithm_CRC16_IBM_Update(uint16_t crc, uint8_t byte)
{
    unsigned int bit;

    crc ^= (uint16_t)byte;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = ((crc & UINT16_C(1)) != 0U)
            ? (uint16_t)((crc >> 1U) ^ UINT16_C(0xA001))
            : (uint16_t)(crc >> 1U);
    }
    return crc;
}

bool Algorithm_CRC16_IBM(
    const uint8_t *bytes,
    size_t length,
    uint16_t initial,
    uint16_t *crc)
{
    uint16_t next = initial;
    size_t index;

    if ((crc == NULL) || ((bytes == NULL) && (length != 0U))) {
        return false;
    }
    for (index = 0U; index < length; ++index) {
        next = Algorithm_CRC16_IBM_Update(next, bytes[index]);
    }
    *crc = next;
    return true;
}

bool Algorithm_CRC16_Modbus(
    const uint8_t *bytes, size_t length, uint16_t *crc)
{
    return Algorithm_CRC16_IBM(bytes, length, UINT16_C(0xFFFF), crc);
}
