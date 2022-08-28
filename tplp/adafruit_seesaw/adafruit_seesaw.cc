#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

#include "picolog/picolog.h"
#include "tplp/adafruit_seesaw/defs.h"

// bring in stuff from defs.h
using namespace ::tplp::seesaw;

namespace tplp {

AdafruitSeesaw* AdafruitSeesaw::Init(I2cDeviceHandle i2c_device) {
  return CHECK_NOTNULL(new AdafruitSeesaw(i2c_device));
}

AdafruitSeesaw::AdafruitSeesaw(I2cDeviceHandle i2c) : i2c_(i2c) {}

}  // namespace tplp