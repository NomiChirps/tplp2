#ifndef TPLP_GRAPHICS_LVGL_INIT_H_
#define TPLP_GRAPHICS_LVGL_INIT_H_

#include "tplp/HX8357/HX8357.h"
#include "tplp/SharpLCD/SharpLCD.h"

namespace tplp {

// Starts FreeRTOS tasks etc and returns.
// Caller retains ownership of `display`.
void InitLvgl(SharpLCD* display);

// Starts FreeRTOS tasks etc and returns.
// Caller retains ownership of `display`.
void InitLvgl(HX8357* display);

// Draws something interesting on a loop.
// Argument is ignored. Does not return.
void RunLvglDemo(void*);

}  // namespace tplp

#endif  // TPLP_GRAPHICS_LVGL_INIT_H_