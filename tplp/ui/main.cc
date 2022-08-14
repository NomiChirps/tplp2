#include "tplp/ui/main.h"

#include "lvgl/lvgl.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/screen_home.h"

namespace tplp {
namespace ui {

void ui_main(TplpInterface* tplp) {
  global_tplp_ = tplp;
  ui_screen_home_create(lv_scr_act());
}

}  // namespace ui
}  // namespace tplp
