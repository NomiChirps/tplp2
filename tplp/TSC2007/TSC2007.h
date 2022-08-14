#ifndef TPLP_TSC2007_TSC2007_H_
#define TPLP_TSC2007_TSC2007_H_

#include <functional>

#include "tplp/I2cController.h"

namespace tplp {

// TSC2007 4-wire resistive touchscreen controller with I2C interface.
class TSC2007 {
 public:
  explicit TSC2007(I2cDeviceHandle device);

  struct TouchInfo {
    int16_t x, y, z1, z2;
  };

  using TouchCallback = std::function<void(const TouchInfo&)>;

  // Call Setup() at least once before any other methods.
  util::Status Setup();

  void StartTask(gpio_pin_t penirq);
  
  // Note that this TouchCallback gives you very raw events. They come from an
  // interrupt pin and will need debouncing.
  //
  // Dragging may produce multiple touch events if the pressure varies
  // significantly along the path, but frequently it does not. You'll need to
  // poll ReadPosition() to get path data in that case.
  //
  // Touch events with z1==z2==0 (zero pressure) sometimes happen after
  // releasing pressure, and sometimes don't. Sometimes they also happen just
  // before the "real" touch event. They usually seem to have garbage x,y data.
  //
  // This will CHECK-fail if called more than once.
  void SetCallback(const TouchCallback& callback);

  util::Status ReadPosition(int16_t* x, int16_t* y, int16_t* z1 = nullptr,
                            int16_t* z2 = nullptr);

  static int16_t min_x() { return 0; }
  static int16_t min_y() { return 0; }
  static int16_t max_x() { return (1 << 12) - 1; }
  static int16_t max_y() { return (1 << 12) - 1; }

 private:
  static void TaskFn(void*);

 private:
  I2cDeviceHandle i2c_;
  TouchCallback callback_;
};

}  // namespace tplp

#endif  // TPLP_TSC2007_TSC2007_H_