#ifndef TPLP_MOTOR_SINE_TABLE_H_
#define TPLP_MOTOR_SINE_TABLE_H_

#include <cstddef>
#include <cstdint>

namespace tplp {

static constexpr size_t kSinTableSize = 256;

// Values from sin(0) inclusive to sin(pi/2) exclusive,
// scaled to the full uint16_t range.
extern const uint16_t kSinTable[kSinTableSize];

}  // namespace tplp

#endif  // TPLP_MOTOR_COSINE_TABLE_H_