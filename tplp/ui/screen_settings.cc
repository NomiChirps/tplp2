#include "tplp/ui/screen_settings.h"

#include "lvgl/lvgl.h"
#include "tplp/ui/screen_home.h"
#include "tplp/ui/settings_i2c_devices.h"
#include "tplp/ui/settings_load_cell.h"
#include "tplp/ui/settings_params.h"
#include "tplp/ui/settings_steppers.h"

static void menu_page_changed_cb(lv_event_t* e);
static void back_clicked(lv_event_t* e);
static lv_obj_t* create_text(lv_obj_t* parent, const char* icon,
                             const char* txt);

static lv_obj_t* global_current_menu_page = NULL;

typedef struct {
  // Set at configuration time
  const char* const icon;
  const char* const txt;
  lv_obj_t* (*const create_section)(lv_obj_t* parent);
  void (*const on_load)() = NULL;
  void (*const on_unload)() = NULL;

  // Set at construction time
  lv_obj_t* menu_page;
} settings_section;

static settings_section sections[] = {
    {.icon = LV_SYMBOL_KEYBOARD,
     .txt = "Params",
     .create_section = ui_settings_params_create,
     .on_load = NULL,
     .on_unload = NULL},
    {.icon = LV_SYMBOL_DOWNLOAD,
     .txt = "Load Cell",
     .create_section = ui_settings_load_cell_create,
     .on_load = ui_settings_load_cell_on_load_cb,
     .on_unload = ui_settings_load_cell_on_unload_cb},
    {.icon = LV_SYMBOL_REFRESH,
     .txt = "Steppers",
     .create_section = ui_settings_steppers_create,
     .on_load = NULL,
     .on_unload = NULL},
    {.icon = LV_SYMBOL_USB,
     .txt = "I2C",
     .create_section = ui_settings_i2c_devices_create},
};

static char root_page_title[] = "Settings";

lv_obj_t* ui_screen_settings_create(lv_obj_t* parent) {
  lv_obj_t* menu = lv_menu_create(parent);

  lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
  lv_obj_add_event_cb(menu, back_clicked, LV_EVENT_CLICKED, menu);
  lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
  lv_obj_center(menu);

  lv_obj_t* root_page = lv_menu_page_create(menu, root_page_title);
  lv_obj_set_style_pad_hor(
      root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0),
      0);
  lv_menu_set_sidebar_page(menu, root_page);

  const int length = sizeof(sections) / sizeof(sections[0]);
  for (int i = 0; i < length; i++) {
    lv_obj_t* section_contents = lv_menu_page_create(menu, NULL);
    lv_obj_remove_style_all(section_contents);
    lv_obj_set_style_pad_hor(
        section_contents,
        lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(section_contents);
    sections[i].menu_page = section_contents;

    lv_obj_t* section_title = lv_menu_section_create(root_page);
    lv_obj_t* content =
        create_text(section_title, sections[i].icon, sections[i].txt);
    lv_menu_set_load_page_event(menu, content, section_contents);
  }
  lv_obj_add_event_cb(menu, menu_page_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // sets the first section to be displayed
  lv_event_send(lv_obj_get_child(
                    lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0),
                LV_EVENT_CLICKED, NULL);

  return menu;
}

static lv_obj_t* create_text(lv_obj_t* parent, const char* icon,
                             const char* txt) {
  lv_obj_t* obj = lv_menu_cont_create(parent);

  lv_obj_t* img = NULL;
  lv_obj_t* label = NULL;

  if (icon) {
    img = lv_img_create(obj);
    lv_img_set_src(img, icon);
  }

  if (txt) {
    label = lv_label_create(obj);
    lv_label_set_text(label, txt);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_flex_grow(label, 1);
  }

  return obj;
}

static settings_section* find_section_by_menu_page(lv_obj_t* menu_page) {
  for (unsigned i = 0; i < sizeof(sections) / sizeof(sections[0]); ++i) {
    if (sections[i].menu_page == menu_page) {
      return &sections[i];
    }
  }
  return NULL;
}

static void load_current_section() {
  if (global_current_menu_page) {
    settings_section* section =
        find_section_by_menu_page(global_current_menu_page);
    if (section) {
      if (section->create_section) {
        section->create_section(global_current_menu_page);
      }
      if (section->on_load) {
        section->on_load();
      }
    }
  }
}

static void delete_children(lv_obj_t* obj) {
  lv_obj_t* child;
  while ((child = lv_obj_get_child(obj, -1))) {
    lv_obj_del(child);
  }
}

static void unload_current_section() {
  if (global_current_menu_page) {
    settings_section* section =
        find_section_by_menu_page(global_current_menu_page);
    if (section && section->on_unload) {
      section->on_unload();
    }
    delete_children(global_current_menu_page);
    global_current_menu_page = nullptr;
  }
}

static void menu_page_changed_cb(lv_event_t* e) {
  unload_current_section();
  global_current_menu_page =
      lv_menu_get_cur_main_page(lv_event_get_current_target(e));
  load_current_section();
}

static void back_clicked(lv_event_t* e) {
  lv_obj_t* obj = lv_event_get_target(e);
  lv_obj_t* menu = static_cast<lv_obj_t*>(lv_event_get_user_data(e));

  if (lv_menu_back_btn_is_root(menu, obj)) {
    unload_current_section();

    lv_obj_t* scr = lv_obj_create(NULL);
    ui_screen_home_create(scr);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);
  }
}
