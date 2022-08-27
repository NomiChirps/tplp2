#include <limits>
#include "tplp/ui/globals.h"
#include "tplp/config/params.h"
#include "lvgl/lvgl.h"
#include "settings_i2c_devices.h"

static lv_obj_t * meter;
static lv_meter_scale_t * meter_scale;
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
static void meter_extra_draw(lv_event_t * e);

static lv_timer_t * update_meter_timer;

void ui_settings_load_cell_on_load_cb() {
    lv_timer_resume(update_meter_timer);
    auto params = global_tplp_->GetLoadCellParams();
    lv_spinbox_set_value(offset_spinbox, params.offset);
    lv_spinbox_set_value(scale_spinbox, params.scale);
}

void ui_settings_load_cell_on_unload_cb() {
    lv_timer_pause(update_meter_timer);
}

static void update_meter_cb(lv_timer_t * timer) {
    static int32_t last_value = 0;
    int32_t value = global_tplp_->GetLoadCellValue();
    if (value != last_value) {
        set_meter_value(value);
    }
}

static void spinbox_changed_cb(lv_event_t * e) {
    tplp::ui::LoadCellParams params =  {
        .offset = lv_spinbox_get_value(offset_spinbox),
        .scale = lv_spinbox_get_value(scale_spinbox),
    };
    if (params.scale != 0) {
        global_tplp_->SetLoadCellParams(params);
    }
}

static void set_meter_value(int32_t v)
{
    lv_meter_set_indicator_value(meter, indic, v);
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
    lv_obj_set_scrollbar_mode(content, LV_SCROLLBAR_MODE_OFF);

    meter_create(content);
    offset_spinbox_create(content);
    scale_spinbox_create(content);

    update_meter_timer = lv_timer_create(update_meter_cb, 100, meter);
    lv_timer_pause(update_meter_timer);

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

    lv_obj_t * dec_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(dec_btn, offset_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    offset_spinbox = lv_spinbox_create(content);
    lv_spinbox_set_range(offset_spinbox, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max());
    lv_spinbox_set_digit_format(offset_spinbox, 8, 0);
    lv_spinbox_step_prev(offset_spinbox);
    lv_obj_set_width(offset_spinbox, 100);
    lv_obj_add_event_cb(offset_spinbox, spinbox_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t * inc_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(inc_btn, offset_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

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

    lv_obj_t * dec_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(dec_btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(dec_btn, scale_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    scale_spinbox = lv_spinbox_create(content);
    lv_spinbox_set_range(scale_spinbox, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max());
    lv_spinbox_set_digit_format(scale_spinbox, 8, 0);
    lv_spinbox_step_prev(scale_spinbox);
    lv_obj_set_width(scale_spinbox, 100);
    lv_obj_add_event_cb(scale_spinbox, spinbox_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t * inc_btn = lv_btn_create(content);
    lv_obj_set_style_bg_img_src(inc_btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(inc_btn, scale_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

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
    meter_scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, meter_scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, meter_scale, 10, 4, 15, lv_color_black(), 10);
    lv_meter_set_scale_range(
        meter, meter_scale, -tplp::params::kLoadCellExpectedRangeAfterScaling,
        tplp::params::kLoadCellExpectedRangeAfterScaling, 270, 135);

    /*Add a needle line indicator*/
    indic = lv_meter_add_needle_line(meter, meter_scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    lv_obj_add_event_cb(meter, meter_extra_draw, LV_EVENT_DRAW_POST, NULL);

    return meter;
}

static void meter_extra_draw(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_draw_ctx_t * draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t scale_area;
    lv_obj_get_content_coords(obj, &scale_area);

    lv_coord_t r_edge = LV_MIN(lv_area_get_width(&scale_area) / 2, lv_area_get_height(&scale_area) / 2);
    lv_point_t scale_center;
    scale_center.x = scale_area.x1 + r_edge;
    scale_center.y = scale_area.y1 + r_edge - ((scale_area.y1 - scale_area.y2)/3);

    lv_draw_label_dsc_t label_draw_dsc;
    lv_draw_label_dsc_init(&label_draw_dsc);
    lv_obj_init_draw_label_dsc(obj, LV_PART_TICKS, &label_draw_dsc);

    lv_point_t label_size;
    lv_txt_get_size(&label_size, "N", label_draw_dsc.font, label_draw_dsc.letter_space, label_draw_dsc.line_space,
                            LV_COORD_MAX, LV_TEXT_FLAG_NONE);

    lv_area_t label_cord;
    label_cord.x1 = scale_center.x - label_size.x / 2;
    label_cord.y1 = scale_center.y - label_size.y / 2;
    label_cord.x2 = label_cord.x1 + label_size.x;
    label_cord.y2 = label_cord.y1 + label_size.y;

    lv_draw_label(draw_ctx, &label_draw_dsc, &label_cord, "N", NULL);
}

