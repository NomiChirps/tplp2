#ifndef TPLP_TPLP_CONFIG_H_
#define TPLP_TPLP_CONFIG_H_

#include "tplp/FreeRTOSConfig.h"
#include "tplp/types.h"

namespace tplp {

struct Pins {
  static constexpr gpio_pin_t LCD_CS = 8;
  static constexpr gpio_pin_t SPI_SCLK = 18;
  static constexpr gpio_pin_t SPI_MOSI = 19;
  static constexpr gpio_pin_t NEOPIXEL = 16;
};

// Indices into the fixed-size FreeRTOS thread-local storage array.
struct ThreadLocalStorage {
  static_assert(configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0);
  static constexpr int kThreadLocalLib = 0;
};

struct TaskPriorities {
  // Just so we don't change it in one place and not update the other.
  static_assert(configMAX_PRIORITIES == 5);

  // Bigger numbers are higher priority.
  // P4 is reserved for system startup and the FreeRTOS timer daemon.
  static constexpr int kSpiManager0 = 3;
  static constexpr int kLvglTimerHandler = 1;
  static constexpr int kLvglDisplayDriver = 1;
  static constexpr int kSharpLcdToggleVcom = 0;
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