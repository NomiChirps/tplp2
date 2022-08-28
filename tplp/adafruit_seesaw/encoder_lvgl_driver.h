#ifndef TPLP_ADAFRUIT_SEESAW_ENCODER_LVGL_DRIVER_H_
#define TPLP_ADAFRUIT_SEESAW_ENCODER_LVGL_DRIVER_H_

#include "lvgl/lvgl.h"
#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

namespace tplp {

lv_indev_t* RegisterInputDevice_SeesawEncoder(SeesawEncoder* encoder);

}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_ENCODER_LVGL_DRIVER_H_