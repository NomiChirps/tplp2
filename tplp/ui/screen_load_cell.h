#ifndef TPLP_UI_SCREEN_LOAD_CELL_H_
#define TPLP_UI_SCREEN_LOAD_CELL_H_

#include "lvgl/lvgl.h"

lv_obj_t* ui_screen_load_cell_create(lv_obj_t* parent);
void ui_screen_load_cell_on_unload_cb();

#endif  // TPLP_UI_SCREEN_LOAD_CELL_H_