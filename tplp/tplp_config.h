#ifndef TPLP_TPLP_CONFIG_H_
#define TPLP_TPLP_CONFIG_H_

#include "tplp/FreeRTOSConfig.h"
#include "tplp/types.h"

namespace tplp {

struct Pins {
  static constexpr gpio_pin_t NEOPIXEL = gpio_pin_t(16);  // hardwired

  static constexpr gpio_pin_t SPI1_SCLK = gpio_pin_t(26);
  static constexpr gpio_pin_t SPI1_MOSI = gpio_pin_t(27);
  static constexpr gpio_pin_t SPI1_MISO = gpio_pin_t(28);
  static constexpr gpio_pin_t HX8357_CS = gpio_pin_t(24);
  static constexpr gpio_pin_t HX8357_DC = gpio_pin_t(25);
};

struct TaskPriorities {
  // Just so we don't change it in one place and not update the other.
  static_assert(configMAX_PRIORITIES == 8);
  // We want to run the timer service daemon at a very high priority.
  static_assert(configTIMER_TASK_PRIORITY == 7);

  // Logging is high priority. This essentially means that *all* LOG() calls
  // from lower-priority tasks will block until the message is flushed.
  static constexpr int kLogging = 6;
  static constexpr int kStartup = 5;

  static constexpr int kSpiManager0 = 3;
  static constexpr int kSpiManager1 = 3;
  static constexpr int kLvglTimerHandler = 1;
  static constexpr int kLvglDisplayDriver = 1;

  // FreeRTOS idle task runs at priority 0.
};

struct TaskStacks {
  static constexpr int kDefault = 1024;  // TODO: deprecate
  static constexpr int kLvglTimerHandler = 2048;
  static constexpr int kLogging = 1024;
};

}  // namespace tplp

#endif  // TPLP_TPLP_CONFIG_H_