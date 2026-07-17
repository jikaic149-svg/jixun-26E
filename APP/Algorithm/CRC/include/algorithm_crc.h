#ifndef ALGORITHM_CRC_H
#define ALGORITHM_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Reflected CRC-16/IBM update: poly 0xA001, refin/refout true. */
uint16_t Algorithm_CRC16_IBM_Update(uint16_t crc, uint8_t byte);

/* The legacy jixun3 crc_16() and crc_modbus() were identical: both used
 * init 0xFFFF, poly 0xA001 and xorout 0. This reentrant implementation keeps
 * that wire result without a lazy mutable lookup table. */
bool Algorithm_CRC16_IBM(
    const uint8_t *bytes,
    size_t length,
    uint16_t initial,
    uint16_t *crc);
bool Algorithm_CRC16_Modbus(
    const uint8_t *bytes, size_t length, uint16_t *crc);

#endif
