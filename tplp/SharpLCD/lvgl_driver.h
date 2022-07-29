#ifndef TPLP_SHARPLCD_LVGL_DRIVER_H_
#define TPLP_SHARPLCD_LVGL_DRIVER_H_

#include <memory>

#include "tplp/SharpLCD/SharpLCD.h"
#include "lvgl/lvgl.h"

namespace tplp {

lv_disp_t* InitAndRegisterDisplayDriver(SharpLCD* display);

}  // namespace


#endif  // TPLP_SHARPLCD_LVGL_DRIVER_H_