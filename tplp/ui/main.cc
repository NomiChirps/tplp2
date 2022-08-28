#include "tplp/ui/main.h"

#include "lvgl/lvgl.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/screen_home.h"

namespace tplp {
namespace ui {

void ui_main(TplpInterface* tplp) {
  global_tplp_ = tplp;
  lv_indev_t* indev = nullptr;
  while ((indev = lv_indev_get_next(indev))) {
    if (indev->driver->type == LV_INDEV_TYPE_ENCODER) {
      lv_log("Found an encoder input device.");
      global_indev_encoder_ = indev;
    }
  }
  ui_screen_home_create(lv_scr_act());
}

}  // namespace ui
}  // namespace tplp
