#ifndef TPLP_MOTOR_COSINE_TABLE_H_
#define TPLP_MOTOR_COSINE_TABLE_H_

#include <cstddef>
#include <cstdint>

namespace tplp {

static constexpr size_t kCosineTableSize = 1024;

// Unsigned values of cosine from 0 to pi/2, offset so that zero is at 32768.
extern const uint16_t kCosineTable[kCosineTableSize];

}  // namespace tplp

#endif  // TPLP_MOTOR_COSINE_TABLE_H_