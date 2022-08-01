#ifndef TPLP_HX8357_HX8357_H_
#define TPLP_HX8357_HX8357_H_

#include "tplp/SpiManager.h"
#include "tplp/types.h"

namespace tplp {

class HX8357 {
 public:
  static constexpr int kNominalMaxSpiFrequency = 16'000'000;

 public:
  HX8357(const HX8357&) = delete;
  HX8357& operator=(const HX8357&) = delete;

  // Caller retains ownership of `spi`. SPI frequency should be no greater
  // than `kNominalMaxSpiFrequency`.
  // `cs`: SPI chip select.
  // `dc`: Data/Command.
  explicit HX8357(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc);

  void Begin();

  // Executes some self-test commands and prints the results to debug output.
  // Returns true if everything looked okay.
  bool SelfTest();

 private:
  SpiManager* const spi_;
  SpiDevice* spi_device_;
  const gpio_pin_t cs_;
  const gpio_pin_t dc_;
};

}  // namespace tplp

#endif  // TPLP_HX8357_HX8357_H_