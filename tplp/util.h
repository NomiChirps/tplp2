#ifndef TPLP_UTIL_H_
#define TPLP_UTIL_H_

#include <chrono>

#include "FreeRTOSConfig.h"

namespace tplp {
namespace duration {
using ticks =
    std::chrono::duration<TickType_t, std::ratio<1, configTICK_RATE_HZ>>;
}  // namespace duration

template <typename Duration>
TickType_t as_ticks(Duration arg) {
  return std::chrono::duration_cast<duration::ticks>(arg).count();
}

}  // namespace tplp

#endif  // TPLP_UTIL_H_