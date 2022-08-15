#ifndef TPLP_SHARPLCD_LVGL_DRIVER_H_
#define TPLP_SHARPLCD_LVGL_DRIVER_H_

#include <memory>

#include "lvgl/lvgl.h"
#include "tplp/SharpLCD/SharpLCD.h"

namespace tplp {

lv_disp_t* RegisterDisplayDriver_SharpLCD(SharpLCD* display, int priority, int stack_depth);

}  // namespace tplp

#endif  // TPLP_SHARPLCD_LVGL_DRIVER_H_