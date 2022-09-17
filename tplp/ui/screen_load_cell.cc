#include "screen_load_cell.h"

#include <limits>

#include "lvgl/lvgl.h"
#include "tplp/config/constants.h"
#include "tplp/config/public_params.h"
#include "tplp/ui/globals.h"

static lv_obj_t* meter;
static lv_obj_t* numeric_display;
static lv_meter_scale_t* meter_scale;
static lv_meter_indicator_t* indic;
static void set_meter_value(int32_t v);
static lv_obj_t* meter_create(lv_obj_t* parent);

static lv_obj_t* offset_spinbox;
static lv_obj_t* scale_spinbox;
static lv_obj_t* offset_spinbox_create(lv_obj_t* parent);
static void offset_spinbox_increment_event_cb(lv_event_t* e);
static void offset_spinbox_decrement_event_cb(lv_event_t* e);
static lv_obj_t* scale_spinbox_create(lv_obj_t* parent);
static void scale_spinbox_increment_event_cb(lv_event_t* e);
static void scale_spinbox_decrement_event_cb(lv_event_t* e);

static lv_timer_t* update_meter_timer;

void ui_screen_load_cell_on_load_cb() {
  lv_timer_resume(update_meter_timer);
  lv_spinbox_set_value(offset_spinbox, PARAM_loadcell_offset.Get());
  lv_spinbox_set_value(scale_spinbox, PARAM_loadcell_scale.Get());
}

void ui_screen_load_cell_on_unload_cb() {
  lv_timer_pause(update_meter_timer);
}

static void update_meter_cb(lv_timer_t* timer) {
  static int32_t last_value = 0;
  int32_t value = global_tplp_->GetLoadCellValue();
  if (value != last_value) {
    set_meter_value(value);
  }
}

static void spinbox_changed_cb(lv_event_t* e) {
  int offset = lv_spinbox_get_value(offset_spinbox);
  int scale = lv_spinbox_get_value(scale_spinbox);
  if (offset != PARAM_loadcell_offset.Get()) {
    PARAM_loadcell_offset.Set(offset);
  }
  if (scale != 0 && scale != PARAM_loadcell_scale.Get()) {
    PARAM_loadcell_scale.Set(scale);
  }
}

static void set_meter_value(int32_t v) {
  lv_meter_set_indicator_value(meter, indic, v);
  lv_label_set_text_fmt(numeric_display, "%ld", static_cast<long int>(v));
  lv_obj_set_style_text_align(numeric_display, LV_TEXT_ALIGN_CENTER, 0);
}

lv_obj_t* ui_screen_load_cell_create(lv_obj_t* parent) {
  lv_obj_t* content = lv_obj_create(parent);
  lv_obj_remove_style_all(content);
  lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));
  lv_obj_set_layout(content, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START);

  meter_create(content);
  offset_spinbox_create(content);
  scale_spinbox_create(content);

  update_meter_timer = lv_timer_create(update_meter_cb, 100, meter);
  lv_timer_pause(update_meter_timer);

  return content;
}

static lv_obj_t* offset_spinbox_create(lv_obj_t* parent) {
  lv_obj_t* content = lv_obj_create(parent);
  lv_obj_remove_style_all(content);
  lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(content, 10, 0);
  lv_obj_set_style_pad_gap(content, 10, 0);
  lv_obj_set_layout(content, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(content, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_SPACE_EVENLY);

  lv_obj_t* label = lv_label_create(content);
  lv_label_set_text(label, "Offset: ");
  lv_obj_set_flex_grow(label, 1);

  lv_obj_t* dec_btn = lv_btn_create(content);
  lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(dec_btn, offset_spinbox_decrement_event_cb, LV_EVENT_ALL,
                      NULL);

  offset_spinbox = lv_spinbox_create(content);
  lv_spinbox_set_range(offset_spinbox, std::numeric_limits<int32_t>::min(),
                       std::numeric_limits<int32_t>::max());
  lv_spinbox_set_digit_format(offset_spinbox, 8, 0);
  lv_spinbox_step_prev(offset_spinbox);
  lv_obj_set_width(offset_spinbox, 100);
  lv_obj_add_event_cb(offset_spinbox, spinbox_changed_cb,
                      LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t* inc_btn = lv_btn_create(content);
  lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(inc_btn, offset_spinbox_increment_event_cb, LV_EVENT_ALL,
                      NULL);

  lv_coord_t h = lv_obj_get_height(offset_spinbox);
  lv_obj_set_size(inc_btn, h, h);
  lv_obj_set_size(dec_btn, h, h);

  return content;
}

static void offset_spinbox_increment_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(offset_spinbox);
  }
}

static void offset_spinbox_decrement_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(offset_spinbox);
  }
}

static lv_obj_t* scale_spinbox_create(lv_obj_t* parent) {
  lv_obj_t* content = lv_obj_create(parent);
  lv_obj_remove_style_all(content);
  lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(content, 10, 0);
  lv_obj_set_style_pad_gap(content, 10, 0);
  lv_obj_set_layout(content, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(content, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_SPACE_EVENLY);

  lv_obj_t* label = lv_label_create(content);
  lv_label_set_text(label, "Scale: ");
  lv_obj_set_flex_grow(label, 1);

  lv_obj_t* dec_btn = lv_btn_create(content);
  lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(dec_btn, scale_spinbox_decrement_event_cb, LV_EVENT_ALL,
                      NULL);

  scale_spinbox = lv_spinbox_create(content);
  lv_spinbox_set_range(scale_spinbox, std::numeric_limits<int32_t>::min(),
                       std::numeric_limits<int32_t>::max());
  lv_spinbox_set_digit_format(scale_spinbox, 8, 0);
  lv_spinbox_step_prev(scale_spinbox);
  lv_obj_set_width(scale_spinbox, 100);
  lv_obj_add_event_cb(scale_spinbox, spinbox_changed_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);

  lv_obj_t* inc_btn = lv_btn_create(content);
  lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(inc_btn, scale_spinbox_increment_event_cb, LV_EVENT_ALL,
                      NULL);

  lv_coord_t h = lv_obj_get_height(scale_spinbox);
  lv_obj_set_size(inc_btn, h, h);
  lv_obj_set_size(dec_btn, h, h);

  return content;
}

static void scale_spinbox_increment_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(scale_spinbox);
  }
}

static void scale_spinbox_decrement_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(scale_spinbox);
  }
}

static lv_obj_t* meter_create(lv_obj_t* parent) {
  meter = lv_meter_create(parent);
  lv_obj_set_size(meter, 180, 180);

  /*Add a scale first*/
  meter_scale = lv_meter_add_scale(meter);
  lv_meter_set_scale_ticks(meter, meter_scale, 41, 2, 5,
                           lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(meter, meter_scale, 10, 4, 10,
                                 lv_color_black(), 15);
  lv_meter_set_scale_range(
      meter, meter_scale, -tplp::constants::kLoadCellExpectedRangeAfterScaling,
      tplp::constants::kLoadCellExpectedRangeAfterScaling, 270, 135);

  /*Add a needle line indicator*/
  indic = lv_meter_add_needle_line(meter, meter_scale, 4,
                                   lv_palette_main(LV_PALETTE_GREY), -10);

  numeric_display = lv_label_create(meter);
  lv_obj_set_size(numeric_display, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_text_align(numeric_display, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align_to(numeric_display, meter, LV_ALIGN_BOTTOM_MID, 0, 0);

  return meter;
}
