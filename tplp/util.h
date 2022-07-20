#include <chrono>

#include "FreeRTOS.h"

namespace tplp {

template <typename Duration>
TickType_t as_ticks(Duration arg) {
  using ticks =
      std::chrono::duration<TickType_t, std::ratio<portTICK_PERIOD_MS, 1000>>;
  return std::chrono::duration_cast<ticks>(arg).count();
}

}  // namespace tplp