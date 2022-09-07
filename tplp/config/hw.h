#ifndef TPLP_CONFIG_PINS_H_
#define TPLP_CONFIG_PINS_H_

#include "pico/config_autogen.h"  // for board identifier
#include "tplp/bus/types.h"

namespace tplp {

// All frequencies are in hertz unless otherwise specified.
struct Frequencies {
  // HX8357 display is on SPI1 and is rated for 16MHz, although it can
  // be overclocked significantly faster.
  static constexpr int kSpi1 = 16'000'000;

  // With I2cController in the disgraceful state it's in,
  // no other value seems to work.
  static constexpr int kI2c0 = 100'000;

  // TODO: see about increasing it. TB6612 wasn't happy @ 100kHz.
  static constexpr int kStepperPwm = 25'000;
};

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
  // GP0 free
  // Each stepper motor's 4 pins must be consecutively numbered.
  static constexpr gpio_pin_t MOTOR_A_A1 = gpio_pin_t(1);
  static constexpr gpio_pin_t MOTOR_A_A2 = gpio_pin_t(2);
  static constexpr gpio_pin_t MOTOR_A_B1 = gpio_pin_t(3);
  static constexpr gpio_pin_t MOTOR_A_B2 = gpio_pin_t(4);
  static constexpr gpio_pin_t MOTOR_B_A1 = gpio_pin_t(5);
  static constexpr gpio_pin_t MOTOR_B_A2 = gpio_pin_t(6);
  static constexpr gpio_pin_t MOTOR_B_B1 = gpio_pin_t(7);
  static constexpr gpio_pin_t MOTOR_B_B2 = gpio_pin_t(8);
  static constexpr gpio_pin_t HX8357_CS = gpio_pin_t(9);
  static constexpr gpio_pin_t SPI1_SCLK = gpio_pin_t(10);
  static constexpr gpio_pin_t SPI1_MOSI = gpio_pin_t(11);
  static constexpr gpio_pin_t SPI1_MISO = gpio_pin_t(12);
  static constexpr gpio_pin_t HX8357_DC = gpio_pin_t(13);
  static constexpr gpio_pin_t HX711_SCK = gpio_pin_t(14);
  static constexpr gpio_pin_t HX711_DOUT = gpio_pin_t(15);
  static constexpr gpio_pin_t I2C0_SDA = gpio_pin_t(16);
  static constexpr gpio_pin_t I2C0_SCL = gpio_pin_t(17);
  // GP18 free
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
  // Adafruit seesaw-based rotary encoder with knob.
  static constexpr i2c_address_t kRotaryEncoder = i2c_address_t(0x36);
};

}  // namespace tplp

#endif  // TPLP_CONFIG_PINS_H_