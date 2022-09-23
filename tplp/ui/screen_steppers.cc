#include "screen_steppers.h"

#include <cstddef>

#include "absl/strings/str_cat.h"
#include "lvgl/lvgl.h"
#include "tplp/config/constants.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/popups.h"

static constexpr int kMaxSpeed = 500;
static constexpr int kExtraSliderClickArea = 50;
static constexpr int kPositionUpdatePeriodMs = 50;

static lv_obj_t* motor_a_speed;
static lv_obj_t* motor_a_speed_label;
static lv_obj_t* motor_a_position;
static lv_obj_t* motor_b_speed;
static lv_obj_t* motor_b_speed_label;
static lv_obj_t* motor_b_position;

static lv_timer_t* update_positions_timer = nullptr;

void stop_button_pressed_cb(lv_event_t* e) {
  util::Status status = global_tplp_->SteppersRelease();
  if (!status.ok()) {
    show_popup_error(std::move(status));
    return;
  }
  lv_slider_set_value(motor_a_speed, 0, LV_ANIM_OFF);
  lv_slider_set_value(motor_b_speed, 0, LV_ANIM_OFF);
  lv_label_set_text(motor_a_speed_label, "-");
  lv_label_set_text(motor_b_speed_label, "-");
}

void update_speeds_cb(lv_event_t* e) {
  int32_t a = lv_slider_get_value(motor_a_speed);
  int32_t b = lv_slider_get_value(motor_b_speed);
  util::Status status = global_tplp_->SteppersSetSpeed(a, b);
  if (!status.ok()) {
    show_popup_error(std::move(status));
    return;
  }
  lv_label_set_text(motor_a_speed_label, absl::StrCat(a).c_str());
  lv_label_set_text(motor_b_speed_label, absl::StrCat(b).c_str());
}

void update_positions_cb(lv_timer_t* timer) {
  int32_t a, b;
  global_tplp_->SteppersGetPosition(&a, &b);
  lv_label_set_text(motor_a_position, absl::StrCat(a).c_str());
  lv_label_set_text(motor_b_position, absl::StrCat(b).c_str());
}

void screen_steppers_delete_cb(lv_event_t* e) {
  lv_timer_pause(update_positions_timer);
}

lv_obj_t* ui_screen_steppers_create(lv_obj_t* parent) {
  static lv_coord_t col_dsc[] = {LV_GRID_CONTENT, LV_GRID_FR(1),
                                 LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {LV_GRID_CONTENT, 100, 100, LV_GRID_FR(1),
                                 LV_GRID_TEMPLATE_LAST};

  lv_obj_t* content = lv_obj_create(parent);
  lv_obj_remove_style_all(content);
  lv_obj_set_style_pad_all(content, 10, 0);
  lv_obj_set_style_pad_gap(content, 10, 0);
  lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));

  lv_obj_set_style_grid_column_dsc_array(content, col_dsc, 0);
  lv_obj_set_style_grid_row_dsc_array(content, row_dsc, 0);
  lv_obj_set_layout(content, LV_LAYOUT_GRID);

  /* Header */
  lv_obj_t* col0_heading = lv_label_create(content);
  lv_label_set_text(col0_heading, "Motor");
  lv_obj_set_grid_cell(col0_heading, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  lv_obj_t* col1_heading = lv_label_create(content);
  lv_label_set_text(col1_heading, "Speed");
  lv_obj_set_grid_cell(col1_heading, LV_GRID_ALIGN_CENTER, 1, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  lv_obj_t* col2_heading = lv_label_create(content);
  lv_label_set_text(col2_heading, "Position");
  lv_obj_set_grid_cell(col2_heading, LV_GRID_ALIGN_CENTER, 2, 1,
                       LV_GRID_ALIGN_CENTER, 0, 1);

  /* Motor 1 */
  lv_obj_t* motor_a_label = lv_label_create(content);
  lv_label_set_text(motor_a_label, "A");
  lv_obj_set_grid_cell(motor_a_label, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);

  motor_a_speed = lv_slider_create(content);
  lv_slider_set_mode(motor_a_speed, LV_SLIDER_MODE_SYMMETRICAL);
  lv_slider_set_range(motor_a_speed, -kMaxSpeed, kMaxSpeed);
  lv_obj_clear_flag(motor_a_speed, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(motor_a_speed, update_speeds_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  lv_obj_set_grid_cell(motor_a_speed, LV_GRID_ALIGN_STRETCH, 1, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);
  lv_obj_set_ext_click_area(motor_a_speed, kExtraSliderClickArea);

  motor_a_speed_label = lv_label_create(content);
  lv_obj_add_flag(motor_a_speed_label, LV_OBJ_FLAG_FLOATING);
  lv_label_set_text(motor_a_speed_label, "0");
  lv_obj_align_to(motor_a_speed_label, motor_a_speed, LV_ALIGN_OUT_BOTTOM_MID,
                  0, 10);

  motor_a_position = lv_label_create(content);
  lv_label_set_text(motor_a_position, "0");
  lv_obj_set_grid_cell(motor_a_position, LV_GRID_ALIGN_CENTER, 2, 1,
                       LV_GRID_ALIGN_CENTER, 1, 1);

  /* Motor 2 */
  lv_obj_t* motor_b_label = lv_label_create(content);
  lv_label_set_text(motor_b_label, "B");
  lv_obj_set_grid_cell(motor_b_label, LV_GRID_ALIGN_CENTER, 0, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);

  motor_b_speed = lv_slider_create(content);
  lv_slider_set_mode(motor_b_speed, LV_SLIDER_MODE_SYMMETRICAL);
  lv_slider_set_range(motor_b_speed, -kMaxSpeed, kMaxSpeed);
  lv_obj_clear_flag(motor_b_speed, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(motor_b_speed, update_speeds_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  lv_obj_set_grid_cell(motor_b_speed, LV_GRID_ALIGN_STRETCH, 1, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);
  lv_obj_set_ext_click_area(motor_b_speed, kExtraSliderClickArea);

  motor_b_speed_label = lv_label_create(content);
  lv_obj_add_flag(motor_b_speed_label, LV_OBJ_FLAG_FLOATING);
  lv_label_set_text(motor_b_speed_label, "0");
  lv_obj_align_to(motor_b_speed_label, motor_b_speed, LV_ALIGN_OUT_BOTTOM_MID,
                  0, 10);

  motor_b_position = lv_label_create(content);
  lv_label_set_text(motor_b_position, "0");
  lv_obj_set_grid_cell(motor_b_position, LV_GRID_ALIGN_CENTER, 2, 1,
                       LV_GRID_ALIGN_CENTER, 2, 1);

  /* play/stop footer buttons */
  lv_obj_t* play_button_bar = lv_obj_create(content);
  lv_obj_remove_style_all(play_button_bar);
  lv_obj_add_flag(play_button_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_pad_all(play_button_bar, 0, 0);
  lv_obj_set_style_pad_gap(play_button_bar, 10, 0);
  lv_obj_set_layout(play_button_bar, LV_LAYOUT_FLEX);
  lv_obj_set_size(play_button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(play_button_bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_grid_cell(play_button_bar, LV_GRID_ALIGN_END, 1, 1,
                       LV_GRID_ALIGN_END, 3, 1);

  lv_obj_t* stop_btn = lv_btn_create(play_button_bar);
  lv_obj_set_style_bg_color(stop_btn, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_t* stop_btn_label = lv_img_create(stop_btn);
  lv_img_set_src(stop_btn_label, LV_SYMBOL_STOP);
  lv_obj_add_event_cb(stop_btn, stop_button_pressed_cb, LV_EVENT_CLICKED, NULL);

  // Background position update
  lv_obj_add_event_cb(content, screen_steppers_delete_cb, LV_EVENT_DELETE,
                      nullptr);
  if (!update_positions_timer) {
    update_positions_timer =
        lv_timer_create(update_positions_cb, kPositionUpdatePeriodMs, nullptr);
  }
  lv_timer_resume(update_positions_timer);

  // lv_obj_t* play_btn = lv_btn_create(play_button_bar);
  // lv_obj_set_style_bg_color(play_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
  // lv_obj_t* play_btn_label = lv_img_create(play_btn);
  // lv_img_set_src(play_btn_label, LV_SYMBOL_PLAY);
  // lv_obj_add_event_cb(play_btn, start_button_pressed_cb, LV_EVENT_CLICKED,
  //                     NULL);

  return content;
}