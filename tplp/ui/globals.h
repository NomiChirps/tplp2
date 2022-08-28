#ifndef TPLP_UI_GLOBALS_H_
#define TPLP_UI_GLOBALS_H_

#include "lvgl/lvgl.h"
#include "tplp/ui/TplpInterface.h"

// A little dirty, but it'll make things a lot easier.
extern tplp::ui::TplpInterface* global_tplp_;

extern lv_indev_t* global_indev_encoder_;

#endif  // TPLP_UI_GLOBALS_H_