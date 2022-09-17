#include "tplp/ui/screen_home.h"

#include "lvgl/lvgl.h"
#include "tplp/ui/globals.h"

static lv_obj_t* ui_contents_create(lv_obj_t* parent);
static void inc_click(lv_event_t* e);
static void update_label(void);

static int counter = 0;
static lv_obj_t* label = NULL;

lv_obj_t* ui_screen_home_create(lv_obj_t* parent) {
  lv_obj_t* root = lv_obj_create(parent);
  lv_obj_set_layout(root, LV_LAYOUT_FLEX);
  lv_obj_set_size(root, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);

  ui_contents_create(root);

  return root;
}

static lv_obj_t* ui_contents_create(lv_obj_t* parent) {
  lv_obj_t* content = lv_obj_create(parent);

  lv_obj_remove_style_all(content);
  lv_obj_set_layout(content, LV_LAYOUT_FLEX);

  lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

  label = lv_label_create(content);
  update_label();

  lv_obj_t* inc_btn = lv_btn_create(content);
  lv_obj_t* inc_label = lv_label_create(inc_btn);
  lv_label_set_text(inc_label, "Go");
  lv_obj_add_event_cb(inc_btn, inc_click, LV_EVENT_CLICKED, NULL);

  return content;
}

static void inc_click(lv_event_t* e) {
  global_tplp_->FlashScreen();
  counter++;
  update_label();
  global_tplp_->RunDevTest();
}

static void update_label(void) {
  lv_label_set_text_fmt(label, "The button has been pressed %d times", counter);
}