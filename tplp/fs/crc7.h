#ifndef CRC7_H_
#define CRC7_H_

#include <cstddef>
#include <cstdint>

uint8_t crc7(uint8_t crc, const uint8_t* buffer, size_t len);

#endif  // CRC7_H_