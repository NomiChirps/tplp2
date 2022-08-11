#ifndef TPLP_HX8357_HX8357_H_
#define TPLP_HX8357_HX8357_H_

#include "tplp/SpiManager.h"
#include "tplp/types.h"

namespace tplp {

class HX8357 {
 public:
  static constexpr int kNominalMaxSpiFrequency = 16'000'000;

 public:
  // Initializes the hardware. It is an error to use any other methods before
  // calling Begin(), or to call Begin() more than once.
  void Begin();

  // Executes a self-diagnostic and prints the results to the INFO log.
  // Returns true if everything looked okay.
  bool SelfTest();

  // DMAs a rectangle of 16-bit 5-6-5 BGR format pixels to a location on the
  // display. Blocks until finished. `x1`, `y1` are the upper-left corner of the
  // rectangle. Crashes if any of the arguments is out of range.
  //
  // `int16_t` was cleverly chosen to match LVGL's default coordinate type.
  void Blit(const uint16_t* pixels, int16_t x1, int16_t y1, int16_t width,
            int16_t height);

  // Returns current width of the display, as modified by the rotation setting.
  int16_t width() const { return display_width_; }
  // Returns current height of the display, as modified by the rotation setting.
  int16_t height() const { return display_height_; }

  // TODO: implement and test rotation setting

 protected:
  enum class DisplayType;
  explicit HX8357(DisplayType type, SpiManager* spi, gpio_pin_t cs,
                  gpio_pin_t dc);

  void SendCommand(uint8_t command, const uint8_t* data = nullptr,
                   uint8_t len = 0);

  // Read Display Self-Diagnostic Result
  uint8_t RDDSDR();

 private:
  void SendInitSequence();

 private:
  const DisplayType type_;
  SpiManager* const spi_;
  SpiDevice* spi_device_;
  const gpio_pin_t cs_;
  const gpio_pin_t dc_;
  // Dimensions as modified by current rotation
  int16_t display_width_;
  int16_t display_height_;

  HX8357(const HX8357&) = delete;
  HX8357& operator=(const HX8357&) = delete;
};

class HX8357B : public HX8357 {
 public:
  // Caller retains ownership of `spi`. SPI frequency should be no greater
  // than `kNominalMaxSpiFrequency`.
  // `cs`: SPI chip select.
  // `dc`: Data/Command.
  explicit HX8357B(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc);
};

class HX8357D : public HX8357 {
 public:
  // Caller retains ownership of `spi`. SPI frequency should be no greater
  // than `kNominalMaxSpiFrequency`.
  // `cs`: SPI chip select.
  // `dc`: Data/Command.
  explicit HX8357D(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc);
};

}  // namespace tplp

#endif  // TPLP_HX8357_HX8357_H_