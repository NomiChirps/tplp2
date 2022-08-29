#include <cstddef>
#include "lvgl/lvgl.h"
#include "settings_steppers.h"
#include "tplp/ui/globals.h"

void start_button_pressed_cb(lv_event_t* e) {
    global_tplp_->RunDevTest();
}

void stop_button_pressed_cb(lv_event_t* e) {
    //
}

static void increment_event_cb(lv_event_t * e);
static void decrement_event_cb(lv_event_t * e);
static void focus_event_cb(lv_event_t * e);

static lv_obj_t * focused_spinner = nullptr;
lv_obj_t * inc_btn;
lv_obj_t * dec_btn;
lv_obj_t * motor_1_led;
lv_obj_t * motor_2_led;

static void set_motor_1_led(bool enabled) {
    if(enabled) {
        lv_led_set_color(motor_1_led, lv_palette_main(LV_PALETTE_RED));
    } else {
        lv_led_set_color(motor_1_led, lv_palette_main(LV_PALETTE_GREEN));
    }
}

static void set_motor_2_led(bool enabled) {
    if(enabled) {
        lv_led_set_color(motor_2_led, lv_palette_main(LV_PALETTE_RED));
    } else {
        lv_led_set_color(motor_2_led, lv_palette_main(LV_PALETTE_GREEN));
    }
}

lv_obj_t * ui_settings_steppers_create(lv_obj_t * parent)
{
    static lv_coord_t col_dsc[] = {LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};

    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_style_pad_all(content, 10, 0);
    lv_obj_set_style_pad_gap(content, 10, 0);
    lv_obj_set_size(content, LV_PCT(100), LV_PCT(95));

    lv_obj_set_style_grid_column_dsc_array(content, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(content, row_dsc, 0);
    lv_obj_set_layout(content, LV_LAYOUT_GRID);

    /* Header */
    lv_obj_t * motor_label = lv_label_create(content);
    lv_label_set_text(motor_label, "Motor");
    lv_obj_set_grid_cell(motor_label, LV_GRID_ALIGN_CENTER, 0, 1,
                             LV_GRID_ALIGN_CENTER, 0, 1);

    lv_obj_t * steps_label = lv_label_create(content);
    lv_label_set_text(steps_label, "Steps");
    lv_obj_set_grid_cell(steps_label, LV_GRID_ALIGN_START, 1, 1,
                             LV_GRID_ALIGN_CENTER, 0, 1);

    lv_obj_t * speed_label = lv_label_create(content);
    lv_label_set_text(speed_label, "Speed");
    lv_obj_set_grid_cell(speed_label, LV_GRID_ALIGN_START, 2, 1,
                             LV_GRID_ALIGN_CENTER, 0, 1);

    /* Motor 1 */
    motor_1_led = lv_led_create(content);
    set_motor_1_led(true);
    lv_obj_t * motor_1_label = lv_label_create(motor_1_led);
    lv_label_set_text(motor_1_label, "1");
    lv_obj_set_size(motor_1_label, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_center(motor_1_label);
    lv_obj_set_style_text_align(motor_1_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_grid_cell(motor_1_led, LV_GRID_ALIGN_CENTER, 0, 1,
                             LV_GRID_ALIGN_CENTER, 1, 1);

    lv_obj_t * motor_1_step_count = lv_spinbox_create(content);
    lv_obj_set_width(motor_1_step_count, 100);
    lv_obj_add_event_cb(motor_1_step_count, focus_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_grid_cell(motor_1_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                        LV_GRID_ALIGN_CENTER, 1, 1);

    lv_obj_t * motor_1_speed = lv_spinbox_create(content);
    lv_obj_set_width(motor_1_speed, 100);
    lv_obj_add_event_cb(motor_1_speed, focus_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_grid_cell(motor_1_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                    LV_GRID_ALIGN_CENTER, 1, 1);

    // /* Motor 2 */
    motor_2_led = lv_led_create(content);
    set_motor_2_led(true);
    lv_obj_t * motor_2_label = lv_label_create(motor_2_led);
    lv_label_set_text(motor_2_label, "2");
    lv_obj_set_size(motor_2_label, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_center(motor_2_label);
    lv_obj_set_style_text_align(motor_2_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_grid_cell(motor_2_led, LV_GRID_ALIGN_CENTER, 0, 1,
                             LV_GRID_ALIGN_CENTER, 2, 1);

    lv_obj_t * motor_2_step_count = lv_spinbox_create(content);
    lv_obj_set_width(motor_2_step_count, 100);
    lv_obj_add_event_cb(motor_2_step_count, focus_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_grid_cell(motor_2_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                        LV_GRID_ALIGN_CENTER, 2, 1);

    lv_obj_t * motor_2_speed = lv_spinbox_create(content);
    lv_obj_set_width(motor_2_speed, 100);
    lv_obj_add_event_cb(motor_2_speed, focus_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_grid_cell(motor_2_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                    LV_GRID_ALIGN_CENTER, 2, 1);


    /* inc/dec footer buttons */
    lv_obj_t * setting_button_bar = lv_obj_create(content);
    lv_obj_remove_style_all(setting_button_bar);
    lv_obj_set_style_pad_all(setting_button_bar, 0, 0);
    lv_obj_set_style_pad_gap(setting_button_bar, 10, 0);
    lv_obj_set_layout(setting_button_bar, LV_LAYOUT_FLEX);
    lv_obj_set_size(setting_button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(setting_button_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_grid_cell(setting_button_bar, LV_GRID_ALIGN_START, 0, 2,
                    LV_GRID_ALIGN_END, 3, 1);

    inc_btn = lv_btn_create(setting_button_bar);
    lv_obj_t * inc_btn_label = lv_img_create(inc_btn);
    lv_img_set_src(inc_btn_label, LV_SYMBOL_PLUS);
    lv_obj_clear_flag(inc_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(inc_btn, increment_event_cb, LV_EVENT_ALL,  NULL);
    lv_obj_add_state(inc_btn, LV_STATE_DISABLED);

    dec_btn = lv_btn_create(setting_button_bar);
    lv_obj_t * dec_btn_label = lv_img_create(dec_btn);
    lv_img_set_src(dec_btn_label, LV_SYMBOL_MINUS);
    lv_obj_clear_flag(dec_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(dec_btn, decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_state(dec_btn, LV_STATE_DISABLED);


    /* play/stop footer buttons */
    lv_obj_t * play_button_bar = lv_obj_create(content);
    lv_obj_remove_style_all(play_button_bar);
    lv_obj_set_style_pad_all(play_button_bar, 0, 0);
    lv_obj_set_style_pad_gap(play_button_bar, 10, 0);
    lv_obj_set_layout(play_button_bar, LV_LAYOUT_FLEX);
    lv_obj_set_size(play_button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(play_button_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_grid_cell(play_button_bar, LV_GRID_ALIGN_END, 2, 1,
                    LV_GRID_ALIGN_END, 3, 1);

    lv_obj_t * stop_btn = lv_btn_create(play_button_bar);
    lv_obj_set_style_bg_color(stop_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_t * stop_btn_label = lv_img_create(stop_btn);
    lv_img_set_src(stop_btn_label, LV_SYMBOL_STOP);
    lv_obj_add_event_cb(stop_btn, stop_button_pressed_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * play_btn = lv_btn_create(play_button_bar);
    lv_obj_set_style_bg_color(play_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_t * play_btn_label = lv_img_create(play_btn);
    lv_img_set_src(play_btn_label, LV_SYMBOL_PLAY);
    lv_obj_add_event_cb(play_btn, start_button_pressed_cb, LV_EVENT_CLICKED, NULL);

    return content;
}

static void focus_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * spinner = lv_event_get_target(e);
    // lv_obj_t * kb = lv_event_get_user_data(e);
    if(code == LV_EVENT_FOCUSED) {
        focused_spinner = spinner;
        lv_obj_add_state(spinner, LV_STATE_FOCUS_KEY);
        lv_obj_clear_state(inc_btn, LV_STATE_DISABLED);
        lv_obj_clear_state(dec_btn, LV_STATE_DISABLED);
    }

    if(code == LV_EVENT_DEFOCUSED) {
        focused_spinner = nullptr;
        lv_obj_clear_state(spinner, LV_STATE_FOCUS_KEY);
        lv_obj_add_state(inc_btn, LV_STATE_DISABLED);
        lv_obj_add_state(dec_btn, LV_STATE_DISABLED);
    }
}

static void increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
        if(focused_spinner != nullptr) {
            lv_spinbox_increment(focused_spinner);
        }
    }
}

static void decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        if(focused_spinner != nullptr) {
            lv_spinbox_decrement(focused_spinner);
        }
    }
}