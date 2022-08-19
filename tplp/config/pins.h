#ifndef TPLP_CONFIG_PINS_H_
#define TPLP_CONFIG_PINS_H_

#include "pico/config_autogen.h"  // for board identifier
#include "tplp/bus/types.h"

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
  static constexpr gpio_pin_t UART_TX = gpio_pin_t(0);  // for log output
  // GP1 free
  static constexpr gpio_pin_t HX8357_CS = gpio_pin_t(2);
  static constexpr gpio_pin_t HX8357_DC = gpio_pin_t(3);
  // GP4 free
  // GP5 free
  // GP6 free
  // GP7 free
  // GP8 free
  // GP9 free
  static constexpr gpio_pin_t SPI1_SCLK = gpio_pin_t(10);
  static constexpr gpio_pin_t SPI1_MOSI = gpio_pin_t(11);
  static constexpr gpio_pin_t SPI1_MISO = gpio_pin_t(12);
  // GP13 free
  // GP14 free
  // GP15 free
  static constexpr gpio_pin_t I2C0_SDA = gpio_pin_t(16);
  static constexpr gpio_pin_t I2C0_SCL = gpio_pin_t(17);
  static constexpr gpio_pin_t TOUCHSCREEN_PENIRQ = gpio_pin_t(18);
  // GP19 free
  // GP20 free
  // GP21 free
  // GP22 free
  // GP23-24 reserved by Pico
  // GP25 == PICO_DEFAULT_LED_PIN
  // GP26 free
  // GP27 free
  // GP28 free
#else
#error "Board not detected"
#endif
};

struct I2cPeripheralAddress {
  // TSC2007 touchscreen reader.
  static constexpr i2c_address_t kTSC2007 = i2c_address_t(0x48);
  // AW9523 GPIO expander.
  static constexpr i2c_address_t kAW9523 = i2c_address_t(0x58);
};

}  // namespace tplp

#endif  // TPLP_CONFIG_PINS_H_