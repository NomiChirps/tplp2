#include "lvgl/lvgl.h"
#include "settings_i2c_devices.h"

static lv_obj_t * meter;
static lv_meter_indicator_t * indic;
static void set_meter_value(int32_t v);
static lv_obj_t * meter_create(lv_obj_t * parent);

static lv_obj_t * offset_spinbox;
static lv_obj_t * scale_spinbox;
static lv_obj_t * offset_spinbox_create(lv_obj_t * parent);
static void offset_spinbox_increment_event_cb(lv_event_t * e);
static void offset_spinbox_decrement_event_cb(lv_event_t * e);
static lv_obj_t * scale_spinbox_create(lv_obj_t * parent);
static void scale_spinbox_increment_event_cb(lv_event_t * e);
static void scale_spinbox_decrement_event_cb(lv_event_t * e);

void ui_settings_load_cell_on_load_cb() {
    // TODO: resume load cell polling timer or whatever
    // probably also load current scale/offset values
    lv_log("\n\n\n\non_load\n\n\n\n");
}

void ui_settings_load_cell_on_unload_cb() {
    // TODO: pause the timer probably
    lv_log("\n\n\n\non_unload\n\n\n\n");
}

static void set_meter_value(int32_t v)
{
    lv_meter_set_indicator_value(meter, indic, v);
}

static int32_t get_offset_value() {
    return lv_spinbox_get_value(offset_spinbox);
}

static int32_t get_scale_value() {
    return lv_spinbox_get_value(scale_spinbox);
}

static void set_offset_value(int32_t value) {
    lv_spinbox_set_value(offset_spinbox, value);
}

static void set_scale_value(int32_t value) {
    lv_spinbox_set_value(offset_spinbox, value);
}

lv_obj_t * ui_settings_load_cell_create(lv_obj_t * parent) {
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_style_pad_all(content, 5, 0);
    lv_obj_set_style_pad_gap(content, 5, 0);
    lv_obj_set_size(content, LV_PCT(100), LV_PCT(100));
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);

    meter_create(content);
    offset_spinbox_create(content);
    scale_spinbox_create(content);

    return content;
}

static lv_obj_t * offset_spinbox_create(lv_obj_t * parent)
{
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(content, 10, 0);
    lv_obj_set_style_pad_gap(content, 10, 0);
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(content, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);

    lv_obj_t * label = lv_label_create(content);
    lv_label_set_text(label, "Offset: ");
    lv_obj_set_flex_grow(label, 1);

    lv_obj_t * inc_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(inc_btn, offset_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

    offset_spinbox = lv_spinbox_create(content);
    lv_spinbox_set_range(offset_spinbox, -1000, 25000);
    lv_spinbox_set_digit_format(offset_spinbox, 5, 2);
    lv_spinbox_step_prev(offset_spinbox);
    lv_obj_set_width(offset_spinbox, 100);

    lv_obj_t * dec_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(dec_btn, offset_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    lv_coord_t h = lv_obj_get_height(offset_spinbox);
    lv_obj_set_size(inc_btn, h, h);
    lv_obj_set_size(dec_btn, h, h);

    return content;
}

static void offset_spinbox_increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(offset_spinbox);
    }
}

static void offset_spinbox_decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(offset_spinbox);
    }
}

static lv_obj_t * scale_spinbox_create(lv_obj_t * parent)
{
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(content, 10, 0);
    lv_obj_set_style_pad_gap(content, 10, 0);
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(content, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);

    lv_obj_t * label = lv_label_create(content);
    lv_label_set_text(label, "Scale: ");
    lv_obj_set_flex_grow(label, 1);

    lv_obj_t * inc_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(inc_btn, scale_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

    scale_spinbox = lv_spinbox_create(content);
    lv_spinbox_set_range(scale_spinbox, -1000, 25000);
    lv_spinbox_set_digit_format(scale_spinbox, 5, 2);
    lv_spinbox_step_prev(scale_spinbox);
    lv_obj_set_width(scale_spinbox, 100);

    lv_obj_t * dec_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(dec_btn, scale_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    lv_coord_t h = lv_obj_get_height(scale_spinbox);
    lv_obj_set_size(inc_btn, h, h);
    lv_obj_set_size(dec_btn, h, h);

    return content;
}

static void scale_spinbox_increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(scale_spinbox);
    }
}

static void scale_spinbox_decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(scale_spinbox);
    }
}

static lv_obj_t * meter_create(lv_obj_t * parent) {
    meter = lv_meter_create(parent);
    lv_obj_center(meter);
    lv_obj_set_size(meter, LV_PCT(50), LV_PCT(50));

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 10, 4, 15, lv_color_black(), 10);
    lv_meter_set_scale_range(meter, scale, -10, 10, 270, 135);

    /*Add a needle line indicator*/
    indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    return meter;
}
