#ifndef TPLP_SHARPLCD_LVGL_DRIVER_H_
#define TPLP_SHARPLCD_LVGL_DRIVER_H_

#include <memory>

#include "lvgl/lvgl.h"
#include "tplp/SharpLCD/SharpLCD.h"

namespace tplp {
namespace sharp_lcd {

lv_disp_t* InitAndRegisterDisplayDriver(SharpLCD* display);

}  // namespace sharp_lcd
}  // namespace tplp

#endif  // TPLP_SHARPLCD_LVGL_DRIVER_H_