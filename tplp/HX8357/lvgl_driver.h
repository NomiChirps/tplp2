#ifndef TPLP_HX8357_LVGL_DRIVER_H_
#define TPLP_HX8357_LVGL_DRIVER_H_

#include "lvgl/lvgl.h"
#include "tplp/HX8357/HX8357.h"

namespace tplp {

lv_disp_t* RegisterDisplayDriver_HX8357(HX8357* display);

}  // namespace tplp

#endif  // TPLP_HX8357_LVGL_DRIVER_H_