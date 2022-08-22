#ifndef TPLP_HX8357_HX8357_H_
#define TPLP_HX8357_HX8357_H_

#include "tplp/bus/SpiController.h"
#include "tplp/bus/types.h"

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
  // rectangle; `x2`, `y2` are the bottom-right corner. The length of `pixels`
  // should be `(x2-x1+1)*(y2-y1+1)`. Crashes if any of the arguments is out of
  // range.
  //
  // `int16_t` was cleverly chosen to match LVGL's default coordinate type.
  void Blit(const uint16_t* pixels, int16_t x1, int16_t y1, int16_t x2,
            int16_t y2);

  // This can set mirroring as well as rotation. Do be careful not to change
  // this to an incompatible width/height mode while LVGL is using it.
  //
  // MX: mirror X if false
  // MY: mirror Y if false
  // MV: swap X/Y if true
  //
  // The initial value is 1,1,0 with dimensions 320x480. Changing MV will cause
  // the return values of width() and height() to change.
  void SetRotation(bool mx, bool my, bool mv);

  // True inverts displayed colors, false displays normal colors.
  // This is fast and can be used to flash the screen as visual feedback.
  void SetInvertedColors(bool invert);

  // Returns current width of the display, as modified by the rotation setting.
  int16_t width() const { return width_; }
  // Returns current height of the display, as modified by the rotation setting.
  int16_t height() const { return height_; }

  // TODO: implement and test rotation setting
  // https://github.com/adafruit/Adafruit_HX8357_Library/blob/master/Adafruit_HX8357.cpp

 protected:
  enum class DisplayType;
  explicit HX8357(DisplayType type, SpiController* spi, gpio_pin_t cs,
                  gpio_pin_t dc);

  // Contract: dc_ must be low before these transfers, and will be low after.
  void SendCommand(SpiTransaction& txn, uint8_t command,
                   const uint8_t* data = nullptr, uint8_t len = 0);

  // Read Display Self-Diagnostic Result
  uint8_t RDDSDR();

 private:
  void SendInitSequence();

 private:
  const DisplayType type_;
  SpiController* const spi_;
  SpiDevice* spi_device_;
  const gpio_pin_t cs_;
  const gpio_pin_t dc_;
  // Dimensions as modified by current rotation
  int16_t width_;
  int16_t height_;

  HX8357(const HX8357&) = delete;
  HX8357& operator=(const HX8357&) = delete;
};

class HX8357B : public HX8357 {
 public:
  // Caller retains ownership of `spi`. SPI frequency should be no greater
  // than `kNominalMaxSpiFrequency`.
  // `cs`: SPI chip select.
  // `dc`: Data/Command.
  explicit HX8357B(SpiController* spi, gpio_pin_t cs, gpio_pin_t dc);
};

class HX8357D : public HX8357 {
 public:
  // Caller retains ownership of `spi`. SPI frequency should be no greater
  // than `kNominalMaxSpiFrequency`.
  // `cs`: SPI chip select.
  // `dc`: Data/Command.
  explicit HX8357D(SpiController* spi, gpio_pin_t cs, gpio_pin_t dc);
};

}  // namespace tplp

#endif  // TPLP_HX8357_HX8357_H_