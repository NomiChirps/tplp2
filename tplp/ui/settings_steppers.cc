#include "lvgl/lvgl.h"
#include "settings_steppers.h"

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
    lv_obj_t * motor_1_label = lv_label_create(content);
    lv_label_set_text(motor_1_label, "1");
    lv_obj_set_grid_cell(motor_1_label, LV_GRID_ALIGN_CENTER, 0, 1,
                             LV_GRID_ALIGN_CENTER, 1, 1);

    lv_obj_t * motor_1_step_count = lv_spinbox_create(content);
    lv_obj_set_width(motor_1_step_count, 100);
    lv_obj_set_grid_cell(motor_1_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                        LV_GRID_ALIGN_CENTER, 1, 1);

    lv_obj_t * motor_1_speed = lv_spinbox_create(content);
    lv_obj_set_width(motor_1_speed, 100);
    lv_obj_set_grid_cell(motor_1_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                    LV_GRID_ALIGN_CENTER, 1, 1);

    // /* Motor 2 */
    lv_obj_t * motor_2_label = lv_label_create(content);
    lv_label_set_text(motor_2_label, "2");
    lv_obj_set_grid_cell(motor_2_label, LV_GRID_ALIGN_CENTER, 0, 1,
                             LV_GRID_ALIGN_CENTER, 2, 1);

    lv_obj_t * motor_2_step_count = lv_spinbox_create(content);
    lv_obj_set_width(motor_2_step_count, 100);
    lv_obj_set_grid_cell(motor_2_step_count, LV_GRID_ALIGN_STRETCH, 1, 1,
                        LV_GRID_ALIGN_CENTER, 2, 1);

    lv_obj_t * motor_2_speed = lv_spinbox_create(content);
    lv_obj_set_width(motor_2_speed, 100);
    lv_obj_set_grid_cell(motor_2_speed, LV_GRID_ALIGN_STRETCH, 2, 1,
                    LV_GRID_ALIGN_CENTER, 2, 1);

    /* footer buttons */
    lv_obj_t * button_bar = lv_obj_create(content);
    lv_obj_remove_style_all(button_bar);
    lv_obj_set_style_pad_all(button_bar, 0, 0);
    lv_obj_set_style_pad_gap(button_bar, 10, 0);
    lv_obj_set_layout(button_bar, LV_LAYOUT_FLEX);
    lv_obj_set_size(button_bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(button_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_grid_cell(button_bar, LV_GRID_ALIGN_END, 2, 1,
                    LV_GRID_ALIGN_END, 3, 1);

    lv_obj_t * stop_btn = lv_btn_create(button_bar);
    lv_obj_set_style_bg_color(stop_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_t * stop_btn_label = lv_img_create(stop_btn);
    lv_img_set_src(stop_btn_label, LV_SYMBOL_STOP);

    lv_obj_t * play_btn = lv_btn_create(button_bar);
    lv_obj_set_style_bg_color(play_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_t * play_btn_label = lv_img_create(play_btn);
    lv_img_set_src(play_btn_label, LV_SYMBOL_PLAY);

    return content;
}
