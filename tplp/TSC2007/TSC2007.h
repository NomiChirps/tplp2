#ifndef TPLP_TSC2007_TSC2007_H_
#define TPLP_TSC2007_TSC2007_H_

#include "tplp/I2cController.h"

namespace tplp {

// TSC2007 4-wire resistive touchscreen controller with I2C interface.
class TSC2007 {
 public:
  explicit TSC2007(I2cDeviceHandle device);

  void StartTask(gpio_pin_t penirq);

  util::Status Setup();
  util::Status ReadPosition(int16_t* x, int16_t* y, int16_t* z1 = nullptr,
                            int16_t* z2 = nullptr);

 private:
  I2cDeviceHandle i2c_;
};

}  // namespace tplp

#endif  // TPLP_TSC2007_TSC2007_H_