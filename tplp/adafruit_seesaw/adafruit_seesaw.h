#ifndef TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_
#define TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_

#include "tplp/bus/i2c.h"
#include "tplp/bus/types.h"

namespace tplp {

class AdafruitSeesaw {
 public:
  AdafruitSeesaw* Init(i2c_address_t addr);

 private:
  AdafruitSeesaw(const AdafruitSeesaw&) = delete;
  const AdafruitSeesaw& operator=(const AdafruitSeesaw&) = delete;
};

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_ADAFRUIT_SEESAW_H_