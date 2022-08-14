#ifndef TPLP_TSC2007_LVGL_DRIVER_H_
#define TPLP_TSC2007_LVGL_DRIVER_H_

#include "lvgl/lvgl.h"
#include "tplp/TSC2007/TSC2007.h"

namespace tplp {

lv_indev_t* RegisterInputDevice_TSC2007(TSC2007* touchscreen);

}  // namespace tplp

#endif  // TPLP_TSC2007_LVGL_DRIVER_H_