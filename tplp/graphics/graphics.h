#ifndef TPLP_GRAPHICS_GRAPHICS_H_
#define TPLP_GRAPHICS_GRAPHICS_H_

#include "tplp/SharpLCD/SharpLCD.h"

namespace tplp {

// Starts FreeRTOS tasks etc and returns.
// Caller retains ownership of `display`.
void InitLvgl(SharpLCD* display);

// Draws something interesting on a loop.
// Argument is ignored. Does not return.
void RunLvglDemo(void*);

}  // namespace tplp

#endif  // TPLP_GRAPIHCS_GRAPHICS_H_