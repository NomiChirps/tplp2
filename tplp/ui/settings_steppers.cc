#include "settings_steppers.h"

#include <cstddef>

#include "lvgl/lvgl.h"
#include "tplp/config/params.h"
#include "tplp/ui/globals.h"

// 5 digits in the spinbox
static constexpr int kMaxSpeed = 99'999;

static lv_obj_t* focused_spinner = nullptr;
lv_obj_t* inc_btn;
lv_obj_t* dec_btn;
lv_obj_t* motor_a_speed;
lv_obj_t* motor_a_step_count;
lv_obj_t* motor_a_led;
lv_obj_t* motor_b_speed;
lv_obj_t* motor_b_step_count;
lv_obj_t* motor_b_led;
lv_obj_t* chart;
lv_chart_series_t* ser;

// TODO: set LEDs on page load? poll motor state? idk
static void set_motor_a_led(bool enabled);
static void set_motor_b_led(bool enabled);

void start_button_pressed_cb(lv_event_t* e) {
  util::Status status;
  status = global_tplp_->StepperMotorSetSpeed(
      lv_spinbox_get_value(motor_a_speed), lv_spinbox_get_value(motor_b_speed));
  if (!status.ok()) {
    // TODO: display error
    return;
  }
  status =
      global_tplp_->StepperMotorMove(lv_spinbox_get_value(motor_a_step_count),
                                     lv_spinbox_get_value(motor_b_step_count));
  if (!status.ok()) {
    // TODO: display error
    return;
  }
  set_motor_a_led(true);
  set_motor_b_led(true);
}

void stop_button_pressed_cb(lv_event_t* e) {
  // TODO: display error if not ok
  global_tplp_->StepperMotorStopAll(
      tplp::ui::TplpInterface::StopType::SHORT_BRAKE);
  set_motor_a_led(false);
  set_motor_b_led(false);
}

// TODO: populate load cell data
static const lv_coord_t loadcell_data[] = {
    -2,   2,    0,    -15,  -39,  -63,  -71,  -68,  -67,  -69,  -84,  -95,
    -104, -107, -108, -107, -107, -107, -107, -114, -118, -117, -112, -100,
    -89,  -83,  -71,  -64,  -58,  -58,  -62,  -62,  -58,  -51,  -46,  -39,
    -27,  -10,  4,    7,    1,    -3,   0,    14,   24,   30,   25,   19,
    13,   7,    12,   15,   18,   21,   13,   6,    9,    8,    17,   19,
    13,   11,   11,   11,   23,   30,   37,   34,   25,   14,   15,   19,
    28,   31,   26,   23,   25,   31,   39,   37,   37,   34,   30,   32,
    22,   29,   31,   33,   37,   23,   13,   7,    2,    4,    -2,   2,
    11,   22,   33,   19,   -1,   -27,  -55,  -67,  -72,  -71,  -63,  -49,
    -18,  35,   113,  230,  369,  525,  651,  722,  730,  667,  563,  454,
    357,  305,  288,  274,  255,  212,  173,  143,  117,  82,   39,   -13,
    -53,  -78,  -91,  -101, -113, -124, -131, -131, -131, -129, -128, -129,
    -125, -123, -123, -129, -139, -148, -153, -159, -166, -183, -205, -227,
    -243, -248, -246, -254, -280, -327, -381, -429, -473, -517, -556, -592,
    -612, -620, -620, -614, -604, -591, -574, -540, -497, -441, -389, -358,
    -336, -313, -284, -222, -167, -114, -70,  -47,  -28,  -4,   12,   38,
    52,   58,   56,   56,   57,   68,   77,   86,   86,   80,   69,   67,
    70,   82,   85,   89,   90,   89,   89,   88,   91,   96,   97,   91,
    83,   78,   82,   88,   95,   96,   105,  106,  110,  102,  100,  96,
    98,   97,   101,  98,   99,   100,  107,  113,  119,  115,  110,  96,
    85,   73,   64,   69,   76,   79};

static void increment_event_cb(lv_event_t* e);
static void decrement_event_cb(lv_event_t* e);
static void focus_event_cb(lv_event_t* e);

static void set_motor_a_led(bool enabled) {
  if (enabled) {
    lv_led_set_color(motor_a_led, lv_palette_main(LV_PALETTE_GREEN));
  } else {
    lv_led_set_color(motor_a_led, lv_palette_main(LV_PALETTE_RED));
  }
}

static void set_motor_b_led(bool enabled) {
  if (enabled) {
    lv_led_set_color(motor_b_led, lv_palette_main(LV_PALETTE_GREEN));
  } else {
    lv_led_set_color(motor_b_led, lv_palette_main(LV_PALETTE_RED));
  }
}

static void set_load_cell_data(lv_coord_t* data, uint32_t count) {
  lv_chart_set_point_count(chart, count);
  lv_chart_set_ext_y_array(chart, ser, data);
}

lv_obj_t* ui_settings_steppers_create(lv_obj_t* parent) {
  static lv_coord_t col_dsc[] = {LV_GRID_CONTENT, LV_GRID_FR(1), LV_GRID_FR(1),
                                 LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {LV_GRID_CONTENT, LV_GRID_CONTENT,
                                 LV_GRID_CONTENT, LV_GRID_FR(1),
                                 LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};

  lv_obj_t* content = lv_obj_create(parent);
  lv_obj_remove_style_all(content);
  lv_obj_set_style_pad_all(content, 10, 0);
  lv_obj_set_style_pad_gap(content, 10, 0);
  lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));

  lv_obj_set_style_grid_column_dsc_array(content, col_dsc, 0);
  lv_obj_set_style_grid_row_dsc_array(content, row_dsc, 0);
  lv_obj_set_layout(content, LV_LAYOUT_GRID);

  /* Header */
  lv_obj_t* motor_label = lv_label_create(content);
  lv_label_set_text(motor_label, "Motor");
  lv_obj_set_grid_cell(motor_label, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  lv_obj_t* steps_label = lv_label_create(content);
  lv_label_set_text(steps_label, "Steps");
  lv_obj_set_grid_cell(steps_label, LV_GRID_ALIGN_CENTER, 1, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  lv_obj_t* speed_label = lv_label_create(content);
  lv_label_set_text(speed_label, "Speed");
  lv_obj_set_grid_cell(speed_label, LV_GRID_ALIGN_CENTER, 2, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  /* Motor 1 */
  motor_a_led = lv_led_create(content);
  lv_led_set_color(motor_a_led, lv_palette_main(LV_PALETTE_YELLOW));
  lv_obj_t* motor_a_label = lv_label_create(motor_a_led);
  lv_label_set_text(motor_a_label, "A");
  lv_obj_set_size(motor_a_label, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_center(motor_a_label);
  lv_obj_set_style_text_align(motor_a_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_grid_cell(motor_a_led, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);

  motor_a_step_count = lv_spinbox_create(content);
  lv_obj_set_style_text_font(motor_a_step_count, &lv_font_unscii_16, 0);
  lv_obj_clear_flag(motor_a_step_count, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_width(motor_a_step_count, 100);
  lv_obj_add_event_cb(motor_a_step_count, focus_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(motor_a_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);

  motor_a_speed = lv_spinbox_create(content);
  lv_spinbox_set_range(motor_a_speed, 0, kMaxSpeed);
  lv_obj_set_style_text_font(motor_a_speed, &lv_font_unscii_16, 0);
  lv_obj_clear_flag(motor_a_speed, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_width(motor_a_speed, 100);
  lv_obj_add_event_cb(motor_a_speed, focus_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(motor_a_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);

  /* Motor 2 */
  motor_b_led = lv_led_create(content);
  lv_led_set_color(motor_b_led, lv_palette_main(LV_PALETTE_YELLOW));
  lv_obj_t* motor_b_label = lv_label_create(motor_b_led);
  lv_label_set_text(motor_b_label, "B");
  lv_obj_set_size(motor_b_label, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_center(motor_b_label);
  lv_obj_set_style_text_align(motor_b_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_grid_cell(motor_b_led, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);

  motor_b_step_count = lv_spinbox_create(content);
  lv_obj_set_style_text_font(motor_b_step_count, &lv_font_unscii_16, 0);
  lv_obj_clear_flag(motor_b_step_count, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_width(motor_b_step_count, 100);
  lv_obj_add_event_cb(motor_b_step_count, focus_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(motor_b_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);

  motor_b_speed = lv_spinbox_create(content);
  lv_spinbox_set_range(motor_b_speed, 0, kMaxSpeed);
  lv_obj_set_style_text_font(motor_b_speed, &lv_font_unscii_16, 0);
  lv_obj_clear_flag(motor_b_speed, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_width(motor_b_speed, 100);
  lv_obj_add_event_cb(motor_b_speed, focus_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(motor_b_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);

  /* load cell chart */
  chart = lv_chart_create(content);
  lv_obj_set_size(chart, LV_PCT(100), 100);
  lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y,
                     -tplp::params::kLoadCellExpectedRangeAfterScaling,
                     tplp::params::kLoadCellExpectedRangeAfterScaling);
  ser = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                            LV_CHART_AXIS_PRIMARY_Y);
  lv_obj_set_grid_cell(chart, LV_GRID_ALIGN_STRETCH, 0, 3, LV_GRID_ALIGN_CENTER,
                       3, 1);

  uint32_t point_count = sizeof(loadcell_data) / sizeof(loadcell_data[0]);
  set_load_cell_data((lv_coord_t*)loadcell_data, point_count);

  /* inc/dec footer buttons */
  lv_obj_t* setting_button_bar = lv_obj_create(content);
  lv_obj_remove_style_all(setting_button_bar);
  lv_obj_add_flag(setting_button_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_pad_all(setting_button_bar, 0, 0);
  lv_obj_set_style_pad_gap(setting_button_bar, 10, 0);
  lv_obj_set_layout(setting_button_bar, LV_LAYOUT_FLEX);
  lv_obj_set_size(setting_button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(setting_button_bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_grid_cell(setting_button_bar, LV_GRID_ALIGN_START, 0, 2,
                       LV_GRID_ALIGN_END, 4, 1);

  inc_btn = lv_btn_create(setting_button_bar);
  lv_obj_t* inc_btn_label = lv_img_create(inc_btn);
  lv_img_set_src(inc_btn_label, LV_SYMBOL_PLUS);
  lv_obj_clear_flag(inc_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(inc_btn, increment_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_state(inc_btn, LV_STATE_DISABLED);

  dec_btn = lv_btn_create(setting_button_bar);
  lv_obj_t* dec_btn_label = lv_img_create(dec_btn);
  lv_img_set_src(dec_btn_label, LV_SYMBOL_MINUS);
  lv_obj_clear_flag(dec_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(dec_btn, decrement_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_state(dec_btn, LV_STATE_DISABLED);

  /* play/stop footer buttons */
  lv_obj_t* play_button_bar = lv_obj_create(content);
  lv_obj_remove_style_all(play_button_bar);
  lv_obj_add_flag(play_button_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_pad_all(play_button_bar, 0, 0);
  lv_obj_set_style_pad_gap(play_button_bar, 10, 0);
  lv_obj_set_layout(play_button_bar, LV_LAYOUT_FLEX);
  lv_obj_set_size(play_button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(play_button_bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_grid_cell(play_button_bar, LV_GRID_ALIGN_END, 2, 1,
                       LV_GRID_ALIGN_END, 4, 1);

  lv_obj_t* stop_btn = lv_btn_create(play_button_bar);
  lv_obj_set_style_bg_color(stop_btn, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_t* stop_btn_label = lv_img_create(stop_btn);
  lv_img_set_src(stop_btn_label, LV_SYMBOL_STOP);
  lv_obj_add_event_cb(stop_btn, stop_button_pressed_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t* play_btn = lv_btn_create(play_button_bar);
  lv_obj_set_style_bg_color(play_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_t* play_btn_label = lv_img_create(play_btn);
  lv_img_set_src(play_btn_label, LV_SYMBOL_PLAY);
  lv_obj_add_event_cb(play_btn, start_button_pressed_cb, LV_EVENT_CLICKED,
                      NULL);

  return content;
}

static void focus_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* spinner = lv_event_get_target(e);
  if (code == LV_EVENT_FOCUSED) {
    focused_spinner = spinner;
    lv_obj_add_state(spinner, LV_STATE_FOCUS_KEY);
    lv_obj_clear_state(inc_btn, LV_STATE_DISABLED);
    lv_obj_clear_state(dec_btn, LV_STATE_DISABLED);
  }

  if (code == LV_EVENT_DEFOCUSED) {
    focused_spinner = nullptr;
    lv_obj_clear_state(spinner, LV_STATE_FOCUS_KEY);
    lv_obj_add_state(inc_btn, LV_STATE_DISABLED);
    lv_obj_add_state(dec_btn, LV_STATE_DISABLED);
  }
}

static void increment_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (focused_spinner != nullptr) {
      lv_spinbox_increment(focused_spinner);
    }
  }
}

static void decrement_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (focused_spinner != nullptr) {
      lv_spinbox_decrement(focused_spinner);
    }
  }
}