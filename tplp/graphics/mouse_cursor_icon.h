#ifndef TPLP_GRAPHICS_MOUSE_CURSOR_ICON_H_
#define TPLP_GRAPHICS_MOUSE_CURSOR_ICON_H_

#include "lvgl/lvgl.h"

// TODO: this doesn't really need to be .c
extern "C" {
extern const uint8_t* mouse_cursor_icon_map;
extern lv_img_dsc_t mouse_cursor_icon;
}

#endif  // TPLP_GRAPHICS_MOUSE_CURSOR_ICON_H_