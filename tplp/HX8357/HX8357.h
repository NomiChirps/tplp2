#ifndef TPLP_HX8357_HX8357_H_
#define TPLP_HX8357_HX8357_H_

#include "tplp/SpiManager.h"
#include "tplp/types.h"

namespace tplp {

class HX8357 {
 public:
  static constexpr int kNominalMaxSpiFrequency = 16'000'000;

 public:
  void Begin();

  // Executes a self-diagnostic and prints the results to the INFO log.
  // Returns true if everything looked okay.
  bool SelfTest();

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
  int width_;
  int height_;

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