#ifndef TPLP_TIME_H_
#define TPLP_TIME_H_

#include <chrono>

#include "FreeRTOSConfig.h"

namespace tplp {
namespace duration {
// From the FreeRTOS docs:
// "If configUSE_16_BIT_TICKS is set to non-zero (true), then TickType_t is
// defined to be an unsigned 16-bit type. If configUSE_16_BIT_TICKS is set to
// zero (false), then TickType_t is defined to be an unsigned 32-bit type."
// Just asserting it for simplicity.
static_assert(configUSE_16_BIT_TICKS == 0);
using ticks =
    std::chrono::duration<uint32_t, std::ratio<1, configTICK_RATE_HZ>>;
}  // namespace duration

// Always rounds towards zero.
template <typename Duration>
uint32_t as_ticks(Duration arg) {
  return std::chrono::duration_cast<duration::ticks>(arg).count();
}

// Always rounds up.
template <typename Duration>
uint32_t as_ticks_ceil(Duration arg) {
  return std::chrono::ceil<duration::ticks>(arg).count();
}

}  // namespace tplp

#endif  // TPLP_TIME_H_