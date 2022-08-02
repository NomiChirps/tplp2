#include "tplp/HX8357/HX8357.h"

// - see commands and init sequence at
//   https://github.com/adafruit/Adafruit_HX8357_Library/blob/master/Adafruit_HX8357.h
// - read diagnostics as in
//   https://github.com/adafruit/Adafruit_HX8357_Library/blob/master/examples/graphicstest/graphicstest.ino
// - intermediate class; see especially startWrite()/endWrite()
//   https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.h
// - see high-level (non-virtual) entry points at
//   https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.h
namespace tplp {
namespace {
//
}  // namespace

HX8357::HX8357(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc)
    : spi_(spi), cs_(cs), dc_(dc) {}

void HX8357::Begin() { this->spi_device_ = spi_->AddDevice(cs_, "HX8357"); }

bool HX8357::SelfTest() {
  // todo
  return false;
}

}  // namespace tplp