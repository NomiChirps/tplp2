#ifndef TPLP_UI_SETTINGS_LOAD_CELL_H_
#define TPLP_UI_SETTINGS_LOAD_CELL_H_

#include "lvgl/lvgl.h"

lv_obj_t * ui_settings_load_cell_create(lv_obj_t * parent);
void ui_settings_load_cell_on_load_cb();
void ui_settings_load_cell_on_unload_cb();

#endif  // TPLP_UI_SETTINGS_LOAD_CELL_H_