#ifndef CRC16_H_
#define CRC16_H_

#include <cstddef>
#include <cstdint>

uint16_t crc16(uint16_t crc, const void* buffer, size_t len);

#endif  // CRC16_H_