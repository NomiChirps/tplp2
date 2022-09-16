#include "tplp/ui/popups.h"

#include "absl/strings/str_cat.h"
#include "lvgl/lvgl.h"

static lv_obj_t* modal_container = nullptr;
static lv_obj_t* keyboard = nullptr;

static void keyboard_cb(lv_event_t* ev) {
  switch (lv_event_get_code(ev)) {
    case LV_EVENT_READY:
    case LV_EVENT_CANCEL:
      lv_obj_clear_state(lv_keyboard_get_textarea(keyboard), LV_STATE_FOCUSED);
      lv_obj_del_async(modal_container);
      modal_container = nullptr;
      keyboard = nullptr;
      break;
    default:
      break;
  }
}

void show_popup_numpad(lv_obj_t* textarea) {
  if (!modal_container) {
    modal_container =
        lv_obj_class_create_obj(&lv_msgbox_backdrop_class, lv_layer_top());
    lv_obj_class_init_obj(modal_container);
    lv_obj_clear_flag(modal_container, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_size(modal_container, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(modal_container, LV_OBJ_FLAG_HIDDEN);
  }
  if (!keyboard) {
    keyboard = lv_keyboard_create(modal_container);
    lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_popovers(keyboard, true);
    lv_obj_add_event_cb(keyboard, keyboard_cb, LV_EVENT_ALL, nullptr);
  }

  // TODO: make the textarea more visible over the modal backdrop
  // TODO: change the symbol for the Ok/Confirm button!

  lv_obj_add_state(textarea, LV_STATE_FOCUSED);
  lv_keyboard_set_textarea(keyboard, textarea);
  lv_obj_clear_flag(modal_container, LV_OBJ_FLAG_HIDDEN);
}

void show_popup_error(util::Status status) {
  static const char* btn_txts[] = {nullptr};
  static std::string text;
  text = absl::StrCat(util::StatusCodeToString(status.code()), ":\n",
                      status.message());
  lv_obj_t* msgbox =
      lv_msgbox_create(nullptr, LV_SYMBOL_WARNING, text.c_str(), btn_txts,
                       /*add_close_btn=*/true);
  lv_obj_center(msgbox);
}
