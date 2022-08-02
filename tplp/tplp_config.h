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

// Indices into the fixed-size FreeRTOS thread-local storage array.
struct ThreadLocalStorage {
  static_assert(configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0);
  static constexpr int kThreadLocalLib = 0;
};

struct TaskPriorities {
  // Just so we don't change it in one place and not update the other.
  static_assert(configMAX_PRIORITIES == 5);

  // The highest priority is reserved for system startup and the
  // FreeRTOS timer daemon.
  static constexpr int kStartup = 4;
  static_assert(configTIMER_TASK_PRIORITY == 4);

  static constexpr int kSpiManager0 = 3;
  static constexpr int kSpiManager1 = 3;
  static constexpr int kLogging = 2;
  static constexpr int kLvglTimerHandler = 1;
  static constexpr int kLvglDisplayDriver = 1;

  // FreeRTOS idle task runs at priority 0.
};

struct TaskStacks {
  static constexpr int kDefault = 1024;
  static constexpr int kLvglTimerHandler = 2048;
};

struct TplpConfig {
  static constexpr int kSpiTransmitQueueDepth = 2;
};

}  // namespace tplp

#endif  // TPLP_TPLP_CONFIG_H_