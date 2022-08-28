#ifndef TPLP_ADAFRUIT_SEESAW_4991_LVGL_DRIVER_H_
#define TPLP_ADAFRUIT_SEESAW_4991_LVGL_DRIVER_H_

#include "lvgl/lvgl.h"
#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

namespace tplp {

lv_indev_t* RegisterInputDevice_Adafruit4991(Adafruit4991* board);

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_4991_LVGL_DRIVER_H_