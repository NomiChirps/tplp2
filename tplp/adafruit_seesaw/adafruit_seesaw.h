#ifndef TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_
#define TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_

#include "picolog/status.h"
#include "tplp/bus/i2c.h"

namespace tplp {

class AdafruitSeesaw {
 public:
  AdafruitSeesaw* Init(I2cDeviceHandle i2c_device);

  // move to an encoder class???
  int32_t getEncoderPosition(uint8_t encoder = 0);
  int32_t getEncoderDelta(uint8_t encoder = 0);
  bool enableEncoderInterrupt(uint8_t encoder = 0);
  bool disableEncoderInterrupt(uint8_t encoder = 0);
  void setEncoderPosition(int32_t pos, uint8_t encoder = 0);

 private:
  AdafruitSeesaw(I2cDeviceHandle i2c);
  util::Status Write32(uint8_t reg, uint8_t func, uint32_t data);
  util::Status Read32(uint8_t reg, uint8_t func, uint32_t* data);

 private:
  I2cDeviceHandle i2c_;

 private:
  AdafruitSeesaw(const AdafruitSeesaw&) = delete;
  const AdafruitSeesaw& operator=(const AdafruitSeesaw&) = delete;
};

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_