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

  template <typename Int>
  util::Status Write(uint8_t reg, uint8_t func, Int data);
  template <typename Int>
  util::StatusOr<Int> Read(uint8_t reg, uint8_t func);

  I2cDeviceHandle i2c_;

 private:
  // Not copyable or movable.
  SeesawBase(const SeesawBase&) = delete;
  const SeesawBase& operator=(const SeesawBase&) = delete;
};

class SeesawEncoder : public SeesawBase {
 public:
  util::StatusOr<int32_t> GetPosition();
  util::StatusOr<int32_t> GetDelta();
  util::Status EnableInterrupt();
  util::Status DisableInterrupt();
  util::Status SetPosition(int32_t pos);

 private:
  explicit SeesawEncoder(I2cDeviceHandle i2c, uint8_t index)
      : SeesawBase(i2c), index_(index) {}
  const uint8_t index_;

  friend class Seesaw;
};

class Seesaw : private SeesawBase {
 public:
  static util::StatusOr<Seesaw*> Init(I2cDeviceHandle i2c_device);

  std::unique_ptr<SeesawEncoder> NewEncoder(uint8_t index = 0);

 private:
  explicit Seesaw(I2cDeviceHandle i2c);
  util::Status Begin();
  util::Status SoftwareReset();
};

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_