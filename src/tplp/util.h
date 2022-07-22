#ifndef TPLP_UTIL_H_
#define TPLP_UTIL_H_

#include <chrono>

#include "FreeRTOS.h"

namespace tplp {

template <typename Duration>
TickType_t as_ticks(Duration arg) {
  using ticks =
      std::chrono::duration<TickType_t, std::ratio<1, configTICK_RATE_HZ>>;
  return std::chrono::duration_cast<ticks>(arg).count();
}

}  // namespace tplp

#endif  // TPLP_UTIL_H_