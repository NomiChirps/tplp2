#include "lvgl/lvgl.h"
#include "tplp/ui/screens/settings.h"
#include "tplp/ui/screens/home_screen.h"

static lv_obj_t * ui_test_content_create(lv_obj_t * parent);
static lv_obj_t * ui_i2c_content_create(lv_obj_t * parent);
static void back_clicked(lv_event_t * e);
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt);
static void inc_click(lv_event_t * e);
static void i2c_refresh_clicked(lv_event_t * e);
static void update_label(void);
static void update_i2c_list(void);

static int counter = 0;
static lv_obj_t * label = NULL;
static lv_obj_t * i2c_devices_label = NULL;

typedef struct {
    char * icon;
    char * txt;
    lv_obj_t * (*create_section)(lv_obj_t * parent);
} settings_section;

static const settings_section sections[] = {
    { LV_SYMBOL_SETTINGS, "Test", ui_test_content_create},
    { LV_SYMBOL_USB, "I2C", ui_i2c_content_create},
};

lv_obj_t * ui_settings_create(lv_obj_t * parent)
{
    lv_obj_t * menu = lv_menu_create(parent);

    lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
    lv_obj_add_event_cb(menu, back_clicked, LV_EVENT_CLICKED, menu);
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t * root_page = lv_menu_page_create(menu, "Settings");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_set_sidebar_page(menu, root_page);

    const int length = sizeof(sections) / sizeof(sections[0]);
    for(int i = 0; i < length; i++) {
        lv_obj_t * section_contents = lv_menu_page_create(menu, NULL);
        lv_obj_set_style_pad_hor(section_contents, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
        lv_menu_separator_create(section_contents);
        sections[i].create_section(section_contents);

        lv_obj_t * section_title = lv_menu_section_create(root_page);
        lv_obj_t * content = create_text(section_title, sections[i].icon, sections[i].txt);
        lv_menu_set_load_page_event(menu, content, section_contents);
    }

    // sets the first section to be displayed
    lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);

    return menu;
}

static lv_obj_t * ui_test_content_create(lv_obj_t * parent)
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

static lv_obj_t * ui_i2c_content_create(lv_obj_t * parent)
{
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);

    lv_obj_set_size(content, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t * button = lv_btn_create(content);
    lv_obj_t * img = lv_img_create(button);
    lv_img_set_src(img, LV_SYMBOL_REFRESH);
    lv_obj_add_event_cb(button, i2c_refresh_clicked, LV_EVENT_CLICKED, NULL);

    lv_obj_t * device_list = lv_obj_create(content);
    lv_obj_set_size(device_list, LV_PCT(100), LV_SIZE_CONTENT);

    i2c_devices_label = lv_label_create(device_list);

    update_i2c_list();

    return content;
}

static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);

    lv_obj_t * img = NULL;
    lv_obj_t * label = NULL;

    if(icon) {
        img = lv_img_create(obj);
        lv_img_set_src(img, icon);
    }

    if(txt) {
        label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_flex_grow(label, 1);
    }

    return obj;
}

static void back_clicked(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * menu = lv_event_get_user_data(e);

    if(lv_menu_back_btn_is_root(menu, obj)) {
        lv_obj_t * scr = lv_obj_create(NULL);
        ui_home_screen_create(scr);

        lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_OUT, 300, 0, true);
    }
}

static void inc_click(lv_event_t * e)
{
    counter++;
    update_label();
}

static void i2c_refresh_clicked(lv_event_t * e)
{
    // TODO: add command to refresh list
}

static void update_label(void)
{
    lv_label_set_text_fmt(label, "The button has been pressed %d times", counter);
}

static void update_i2c_list(void)
{
    lv_label_set_text_fmt(i2c_devices_label, "I2C Device 1\nI2C Device 2\nI2C Device 3");
}
