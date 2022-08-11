#ifndef TPLP_HX8357_LVGL_DRIVER_H_
#define TPLP_HX8357_LVGL_DRIVER_H_

#include "lvgl/lvgl.h"
#include "tplp/HX8357/HX8357.h"

namespace tplp {

// This driver makes use of the 0th task notification index on the
// lv_timer_handler task.
lv_disp_t* RegisterDisplayDriver_HX8357(HX8357* display,
                                        TaskHandle_t lv_timer_task);

}  // namespace tplp

#endif  // TPLP_HX8357_LVGL_DRIVER_H_