#ifndef TPLP_GRAPHICS_LVGL_INIT_H_
#define TPLP_GRAPHICS_LVGL_INIT_H_

#include "tplp/hx8357/hx8357.h"
#include "tplp/SharpLCD/SharpLCD.h"
#include "tplp/tsc2007/tsc2007.h"
#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

namespace tplp {

class LvglInit {
 public:
  explicit LvglInit();

  // Call this first, then set devices, then Start().
  // Parameters specify the priority and stack depth of the LVGL timer task.
  void BaseInit(int priority, int stack_depth);

  // Adding multiple displays not tested...
  void SetDisplay(SharpLCD* display, int priority, int stack_depth);
  void SetDisplay(HX8357* display, int priority, int stack_depth);

  // Add at least one display before adding any input devices.
  void AddTouchscreen(TSC2007* touchscreen);
  void AddEncoder(Adafruit4991* encoder);

  void Start();

 private:
  struct Objects;
  Objects* stuff_;
};

}  // namespace tplp

#endif  // TPLP_GRAPHICS_LVGL_INIT_H_