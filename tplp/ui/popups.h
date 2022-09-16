#ifndef TPLP_UI_POPUPS_H_
#define TPLP_UI_POPUPS_H_

#include "lvgl/lvgl.h"
#include "picolog/status.h"

// Sends LV_EVENT_READY or LV_EVENT_CANCEL to the textarea when dismissed.
void show_popup_numpad(lv_obj_t* textarea);

// Simple modal dialog displaying the status's code and message.
void show_popup_error(util::Status status);

#endif  // TPLP_UI_POPUPS_H_