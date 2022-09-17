#include "tplp/ui/main.h"

#include "lvgl/lvgl.h"
#include "picolog/picolog.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/screen_home.h"
#include "tplp/ui/settings_i2c_devices.h"
#include "tplp/ui/settings_load_cell.h"
#include "tplp/ui/settings_params.h"
#include "tplp/ui/settings_steppers.h"

namespace tplp {
namespace ui {
namespace {

// TODO: maybe use virtual dispatch idk
struct ScreenInfo {
  // Set at configuration time
  const char* const label;
  lv_obj_t* (*const create)(lv_obj_t* parent);
  // TODO: screens could just do this in create()
  void (*const on_load)() = nullptr;
  // TODO: screens could just listen for LV_EVENT_DELETED
  void (*const on_unload)() = nullptr;

  // Set at runtime
  lv_obj_t* contents = nullptr;
} static screens[] = {
    {.label = LV_SYMBOL_HOME " Home",  //
     .create = ui_screen_home_create},
    {.label = LV_SYMBOL_KEYBOARD " Params",  //
     .create = ui_settings_params_create},
    {.label = LV_SYMBOL_DOWNLOAD " Load Cell",  //
     .create = ui_settings_load_cell_create,
     .on_load = ui_settings_load_cell_on_load_cb,
     .on_unload = ui_settings_load_cell_on_unload_cb},
    {.label = LV_SYMBOL_REFRESH " Steppers",  //
     .create = ui_settings_steppers_create},
    {.label = LV_SYMBOL_USB " I2C",  //
     .create = ui_settings_i2c_devices_create},
};
static constexpr int kNumScreens = sizeof(screens) / sizeof(screens[0]);

static constexpr int kScreensMenuWidth = 150;
static constexpr int kScreensMenuButtonHeight = 40;

static constexpr int kSidebarWidth = 30;
static constexpr int kSidebarButtonHeight = 30;
static constexpr int kSidebarButtonExtClickArea = 30;

static ScreenInfo* current_screen = nullptr;
static lv_obj_t* screens_menu = nullptr;
static lv_obj_t* screen_contents_container = nullptr;

static void go_to_screen(int index) {
  CHECK_GE(index, 0);
  CHECK_LT(index, kNumScreens);
  lv_obj_add_flag(screens_menu, LV_OBJ_FLAG_HIDDEN);
  if (current_screen == &screens[index]) {
    return;
  }

  if (current_screen) {
    if (current_screen->on_unload) {
      current_screen->on_unload();
    }
    if (current_screen->contents) {
      lv_obj_del(current_screen->contents);
      current_screen->contents = nullptr;
    }
    current_screen = nullptr;
  }

  current_screen = &screens[index];
  current_screen->contents = current_screen->create(screen_contents_container);
  if (current_screen->on_load) {
    current_screen->on_load();
  }
}

static void screens_menu_button_clicked(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    int button_id = lv_btnmatrix_get_selected_btn(lv_event_get_target(e));
    go_to_screen(button_id);
  }
}

static void screens_menu_background_clicked(lv_event_t* e) {
  lv_obj_add_flag(screens_menu, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t* screens_menu_create() {
  static lv_style_t style_bg;
  lv_style_init(&style_bg);
  lv_style_set_pad_all(&style_bg, 0);
  lv_style_set_pad_gap(&style_bg, 0);
  lv_style_set_border_width(&style_bg, 0);
  lv_style_set_width(&style_bg, kScreensMenuWidth);
  lv_style_set_height(&style_bg, kScreensMenuButtonHeight * kNumScreens);
  lv_style_set_radius(&style_bg, 0);

  static lv_style_t style_btn;
  lv_style_init(&style_btn);
  lv_style_set_border_width(&style_btn, 1);
  lv_style_set_radius(&style_btn, 0);

  lv_obj_t* container = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(container);
  lv_obj_set_size(container, lv_disp_get_hor_res(nullptr),
                  lv_disp_get_ver_res(nullptr));
  lv_obj_add_flag(container, LV_OBJ_FLAG_FLOATING);
  lv_obj_set_style_x(container, kSidebarWidth, 0);
  lv_obj_add_event_cb(container, screens_menu_background_clicked,
                      LV_EVENT_CLICKED, nullptr);

  lv_obj_t* buttons = lv_btnmatrix_create(container);
  lv_btnmatrix_set_one_checked(buttons, true);
  lv_obj_add_style(buttons, &style_bg, LV_PART_MAIN);
  lv_obj_add_style(buttons, &style_btn, LV_PART_ITEMS);

  static const char* button_map[2 * kNumScreens];
  for (int i = 0; i < kNumScreens; ++i) {
    button_map[2 * i] = screens[i].label;
    button_map[2 * i + 1] = "\n";
  }
  button_map[2 * kNumScreens - 1] = nullptr;
  lv_btnmatrix_set_map(buttons, button_map);
  lv_btnmatrix_set_btn_ctrl_all(buttons, LV_BTNMATRIX_CTRL_NO_REPEAT |
                                             LV_BTNMATRIX_CTRL_CHECKABLE |
                                             LV_BTNMATRIX_CTRL_CLICK_TRIG);
  lv_btnmatrix_set_btn_ctrl(buttons, 0, LV_BTNMATRIX_CTRL_CHECKED);
  lv_obj_add_event_cb(buttons, screens_menu_button_clicked,
                      LV_EVENT_VALUE_CHANGED, nullptr);

  return container;
}

void sidebar_button_clicked(lv_event_t* e) {
  if (lv_obj_has_flag(screens_menu, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_clear_flag(screens_menu, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(screens_menu, LV_OBJ_FLAG_HIDDEN);
  }
}

lv_obj_t* sidebar_create(lv_obj_t* parent) {
  lv_obj_t* sidebar = lv_obj_create(parent);
  lv_obj_remove_style_all(sidebar);
  lv_obj_set_size(sidebar, kSidebarWidth, lv_disp_get_ver_res(nullptr));

  lv_obj_t* btn = lv_btn_create(sidebar);
  lv_obj_remove_style_all(btn);
  lv_obj_set_size(btn, LV_PCT(100), kSidebarButtonHeight);
  lv_obj_set_style_border_side(btn, LV_BORDER_SIDE_INTERNAL, 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_t* btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, LV_SYMBOL_SETTINGS);
  lv_obj_add_event_cb(btn, sidebar_button_clicked, LV_EVENT_CLICKED, nullptr);
  lv_obj_center(btn_label);

  lv_obj_set_ext_click_area(btn, kSidebarButtonExtClickArea);

  lv_obj_set_style_bg_color(sidebar, lv_color_make(0xcf, 0xcf, 0xcf), 0);
  lv_obj_set_style_bg_opa(sidebar, LV_OPA_100, 0);

  return sidebar;
}

}  // namespace

void ui_main(TplpInterface* tplp) {
  global_tplp_ = tplp;
  if (0) {
    // TODO: encoder stuff doesn't work
    lv_indev_t* indev = nullptr;
    while ((indev = lv_indev_get_next(indev))) {
      if (indev->driver->type == LV_INDEV_TYPE_ENCODER) {
        LV_LOG_USER("Found an encoder input device.");
        global_indev_encoder_ = indev;
      }
    }
  }

  // Creates a floating menu
  screens_menu = screens_menu_create();

  lv_obj_t* toplevel = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(toplevel);
  lv_obj_set_layout(toplevel, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(toplevel, LV_FLEX_FLOW_ROW);
  lv_obj_set_size(toplevel, lv_disp_get_hor_res(nullptr),
                  lv_disp_get_ver_res(nullptr));
  lv_obj_t* sidebar = sidebar_create(toplevel);
  (void)sidebar;  // unused (for now?)

  screen_contents_container = lv_obj_create(toplevel);
  lv_obj_remove_style_all(screen_contents_container);
  lv_obj_set_flex_grow(screen_contents_container, 1);
  lv_obj_set_style_height(screen_contents_container, LV_PCT(100), 0);

  go_to_screen(0);
}

}  // namespace ui
}  // namespace tplp
