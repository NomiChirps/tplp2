#ifndef TPLP_TPLP_CONFIG_H_
#define TPLP_TPLP_CONFIG_H_

#include "FreeRTOSConfig.h"
#include "pico/config_autogen.h"  // for board identifier
#include "tplp/types.h"

namespace tplp {

struct Pins {
#if defined(ADAFRUIT_FEATHER_RP2040)
  // See also: https://learn.adafruit.com/adafruit-feather-rp2040-pico/pinouts
  // Numeric labels on the board do NOT necessarily match the GPIO number!
  // GPIOs 4, 5, 14, 15, 17, 21, 22, 23 are reserved by the platform.
  static constexpr gpio_pin_t NEOPIXEL = gpio_pin_t(16);  // hardwired
  static constexpr gpio_pin_t SPI1_SCLK = gpio_pin_t(26);
  static constexpr gpio_pin_t SPI1_MOSI = gpio_pin_t(27);
  static constexpr gpio_pin_t SPI1_MISO = gpio_pin_t(28);
  static constexpr gpio_pin_t HX8357_CS = gpio_pin_t(24);
  static constexpr gpio_pin_t HX8357_DC = gpio_pin_t(25);
#elif defined(RASPBERRYPI_PICO)
  static constexpr gpio_pin_t SPI1_SCLK = gpio_pin_t(10);
  static constexpr gpio_pin_t SPI1_MOSI = gpio_pin_t(11);
  static constexpr gpio_pin_t SPI1_MISO = gpio_pin_t(12);
  static constexpr gpio_pin_t HX8357_CS = gpio_pin_t(2);
  static constexpr gpio_pin_t HX8357_DC = gpio_pin_t(3);
  static constexpr gpio_pin_t UART_TX = gpio_pin_t(0);  // for log output
  static constexpr gpio_pin_t I2C0_SCL = gpio_pin_t(17);
  static constexpr gpio_pin_t I2C0_SDA = gpio_pin_t(16);
  static constexpr gpio_pin_t TEST_PUSHBUTTON = gpio_pin_t(28);
  static constexpr gpio_pin_t TOUCHSCREEN_PENINT = gpio_pin_t(18);
#else
#error "Board not detected"
#endif
};

struct TaskPriorities {
  // Just so we don't change it in one place and not update the other.
  static_assert(configMAX_PRIORITIES == 8);
  // We want to run the timer service daemon at a very high priority.
  static_assert(configTIMER_TASK_PRIORITY == 7);

  // Logging is high priority. This means that *all* LOG() calls
  // from lower-priority tasks will block until the message is flushed.
  // Maybe later we can reduce this, but for debugging it's essential.
  static constexpr int kLogging = 6;

  // Allow startup to finish without anything else getting in the way.
  // Except logging, of course.
  static constexpr int kStartup = 5;

  // Bus controllers of high quality and esteemed stature.
  static constexpr int kSpiController0 = 4;
  static constexpr int kSpiController1 = 4;

  static constexpr int kRuntimeStats = 2;  // don't care

  // User-interface peripherals are low priority, but not the lowest.
  static constexpr int kSharpLCD = 2;
  static constexpr int kHX8357 = 2;
  static constexpr int kTSC2007 = 2;
  // I2cController needs work; it busy-waits a lot.
  static constexpr int kI2cController0 = 2;

  // Any remaining CPU time goes into making the UI more responsive.
  static constexpr int kLvglTimerHandler = 1;

  // FreeRTOS idle task runs at priority 0.
};

struct TaskStacks {
  static constexpr int kTESTONLY = 1024;
  [[deprecated]] static constexpr int kDefault = 1024;
  static constexpr int kLvglTimerHandler = 2048;
  static constexpr int kHX8357 = 512;
  static constexpr int kI2cController = 1024;
  static constexpr int kSpiController = 1024;
  static constexpr int kRuntimeStats = 512;
  static constexpr int kStartup = 1024;
  static constexpr int kTSC2007 = 512;

  // don't reduce further unless you enjoy heisenbugs
  static constexpr int kLogging = 1024;

  // unused
  static constexpr int kSharpLCD = 1024;
};

struct I2cPeripheralAddress {
  // TSC2007 touchscreen reader.
  static constexpr i2c_address_t kTSC2007 = i2c_address_t(0x48);
};

}  // namespace tplp

#endif  // TPLP_TPLP_CONFIG_H_