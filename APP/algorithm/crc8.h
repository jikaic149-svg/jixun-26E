/**
 * @file    crc8.h
 * @brief   8位CRC校验
 * @origin  云台 modules/algorithm/crc8.h
 * @note    100%可移植
 */
#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

#define CRC_START_8 0x00

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif /* CRC8_H */
