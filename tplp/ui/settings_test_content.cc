#include "lvgl/lvgl.h"
#include "settings_test_content.h"

static void inc_click(lv_event_t * e);
static void update_label(void);

static int counter = 0;
static lv_obj_t * label = NULL;

lv_obj_t * ui_settings_test_content_create(lv_obj_t * parent)
{
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);

    lv_obj_set_size(content, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    label = lv_label_create(content);
    update_label();

    lv_obj_t * inc_btn = lv_btn_create(content);
    lv_obj_t * inc_label = lv_label_create(inc_btn);
    lv_label_set_text(inc_label, "+1");
    lv_obj_add_event_cb(inc_btn, inc_click, LV_EVENT_CLICKED, NULL);

    return content;
}

static void inc_click(lv_event_t * e)
{
    counter++;
    update_label();
}

static void update_label(void)
{
    lv_label_set_text_fmt(label, "The button has been pressed %d times", counter);
}
