#ifndef TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_
#define TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_

#include "picolog/status.h"
#include "picolog/statusor.h"
#include "tplp/bus/i2c.h"

namespace tplp {

class Seesaw;

class SeesawBase {
 protected:
  explicit SeesawBase(I2cDeviceHandle i2c) : i2c_(i2c) {}

  util::Status Init();

  template <typename Int>
  util::Status Write(uint8_t reg, uint8_t func, Int data);
  template <typename Int>
  util::StatusOr<Int> Read(uint8_t reg, uint8_t func);

  util::StatusOr<uint32_t> GetVersion();
  util::StatusOr<uint32_t> GetOptions();
  util::Status SoftwareReset();

  // GPIO
  enum class PinMode : uint8_t {
    INPUT = 0x0,
    OUTPUT = 0x1,
    INPUT_PULLUP = 0x2,
    INPUT_PULLDOWN = 0x4
  };
  util::Status SetPinMode(uint8_t pin, PinMode mode);
  util::Status SetPinModes(uint32_t pins_mask, PinMode mode);
  util::Status SetGpioInterruptsEnabled(uint32_t pins_mask, bool enabled);
  util::StatusOr<bool> DigitalRead(uint8_t pin);
  util::StatusOr<uint32_t> DigitalReadMask(uint32_t pin_mask);

 protected:
  I2cDeviceHandle i2c_;

 private:
  // Not copyable or movable.
  SeesawBase(const SeesawBase&) = delete;
  const SeesawBase& operator=(const SeesawBase&) = delete;
};

class SeesawEncoder : virtual public SeesawBase {
 public:
  // Also clears the interrupt pin, if enabled.
  util::StatusOr<int32_t> GetPosition();
  // Also clears the interrupt pin, if enabled.
  util::StatusOr<int32_t> GetDelta();

  util::Status EnableInterruptPin();
  util::Status DisableInterruptPin();
  util::Status SetPosition(int32_t pos);

 protected:
  explicit SeesawEncoder(I2cDeviceHandle i2c, uint8_t index)
      : SeesawBase(i2c), index_(index) {}

 private:
  const uint8_t index_;
};

class SeesawNeopixel : virtual public SeesawBase {
 public:
  enum class Type : uint8_t {
    // RGB NeoPixel permutations; white and red offsets are always same
    // Offset:  W          R          G          B
    NEO_RGB = ((0 << 6) | (0 << 4) | (1 << 2) | (2)),
    NEO_RBG = ((0 << 6) | (0 << 4) | (2 << 2) | (1)),
    NEO_GRB = ((1 << 6) | (1 << 4) | (0 << 2) | (2)),
    NEO_GBR = ((2 << 6) | (2 << 4) | (0 << 2) | (1)),
    NEO_BRG = ((1 << 6) | (1 << 4) | (2 << 2) | (0)),
    NEO_BGR = ((2 << 6) | (2 << 4) | (1 << 2) | (0)),

    // RGBW NeoPixel permutations; all 4 offsets are distinct
    // Offset:   W          R          G          B
    NEO_WRGB = ((0 << 6) | (1 << 4) | (2 << 2) | (3)),
    NEO_WRBG = ((0 << 6) | (1 << 4) | (3 << 2) | (2)),
    NEO_WGRB = ((0 << 6) | (2 << 4) | (1 << 2) | (3)),
    NEO_WGBR = ((0 << 6) | (3 << 4) | (1 << 2) | (2)),
    NEO_WBRG = ((0 << 6) | (2 << 4) | (3 << 2) | (1)),
    NEO_WBGR = ((0 << 6) | (3 << 4) | (2 << 2) | (1)),

    NEO_RWGB = ((1 << 6) | (0 << 4) | (2 << 2) | (3)),
    NEO_RWBG = ((1 << 6) | (0 << 4) | (3 << 2) | (2)),
    NEO_RGWB = ((2 << 6) | (0 << 4) | (1 << 2) | (3)),
    NEO_RGBW = ((3 << 6) | (0 << 4) | (1 << 2) | (2)),
    NEO_RBWG = ((2 << 6) | (0 << 4) | (3 << 2) | (1)),
    NEO_RBGW = ((3 << 6) | (0 << 4) | (2 << 2) | (1)),

    NEO_GWRB = ((1 << 6) | (2 << 4) | (0 << 2) | (3)),
    NEO_GWBR = ((1 << 6) | (3 << 4) | (0 << 2) | (2)),
    NEO_GRWB = ((2 << 6) | (1 << 4) | (0 << 2) | (3)),
    NEO_GRBW = ((3 << 6) | (1 << 4) | (0 << 2) | (2)),
    NEO_GBWR = ((2 << 6) | (3 << 4) | (0 << 2) | (1)),
    NEO_GBRW = ((3 << 6) | (2 << 4) | (0 << 2) | (1)),

    NEO_BWRG = ((1 << 6) | (2 << 4) | (3 << 2) | (0)),
    NEO_BWGR = ((1 << 6) | (3 << 4) | (2 << 2) | (0)),
    NEO_BRWG = ((2 << 6) | (1 << 4) | (3 << 2) | (0)),
    NEO_BRGW = ((3 << 6) | (1 << 4) | (2 << 2) | (0)),
    NEO_BGWR = ((2 << 6) | (3 << 4) | (1 << 2) | (0)),
    NEO_BGRW = ((3 << 6) | (2 << 4) | (1 << 2) | (0)),
  };

 protected:
  explicit SeesawNeopixel(I2cDeviceHandle i2c, uint16_t num_leds, uint8_t pin,
                          Type type, bool khz800);
  util::Status Init();

 private:
  util::Status UpdateType();
  util::Status UpdateLength(uint16_t num_leds);
  util::Status SetPin(uint8_t pin);

 private:
  uint16_t num_leds_;
  uint8_t pin_;
  const Type type_;
  const bool khz800_;
  uint8_t r_offset_;
  uint8_t g_offset_;
  uint8_t b_offset_;
  uint8_t w_offset_;

  uint8_t* pixels_;
  uint16_t num_bytes_;
};

class Adafruit4991 : public SeesawEncoder, public SeesawNeopixel {
 public:
  explicit Adafruit4991(I2cDeviceHandle i2c);
  util::Status Init();

  // Returns true if currently pressed, false if currently released. This also
  // clears the SeesawEncoder interrupt pin, if enabled.
  util::StatusOr<bool> GetSwitchState();

 private:
  static constexpr uint32_t kExpectedFirmwareVersion = 4991;
  static constexpr int kSwitchPin = 24;
  static constexpr int kNeopixelPin = 6;
  friend class Seesaw;
};

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_