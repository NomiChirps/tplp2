#include "tplp/rtos_util.h"

#include "pico/time.h"

namespace tplp {

void EnsureTimeSinceBootMillis(int wait_until) {
  absolute_time_t now = get_absolute_time();
  int ms_since_boot = to_ms_since_boot(now);
  if (ms_since_boot < wait_until) {
    vTaskDelay(pdMS_TO_TICKS(wait_until - ms_since_boot));
  }
}

}  // namespace tplp