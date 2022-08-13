#ifndef TPLP_TSC2007_TSC2007_H_
#define TPLP_TSC2007_TSC2007_H_

#include "tplp/I2cController.h"

namespace tplp {

// TSC2007 4-wire resistive touchscreen controller with I2C interface.
class TSC2007 {
 public:
  explicit TSC2007(I2cDeviceHandle device);

 private:
};

}  // namespace tplp

#endif  // TPLP_TSC2007_TSC2007_H_