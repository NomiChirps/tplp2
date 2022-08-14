#include "lvgl/lvgl.h"
#include "settings_i2c_devices.h"
#include "tplp/ui/globals.h"

#include <ostream>
#include <iomanip>

using tplp::ui::I2cScanResult;

static void i2c_refresh_clicked(lv_event_t * e);
static void update_i2c_list(const I2cScanResult&);

static lv_obj_t * i2c_devices_label = NULL;

lv_obj_t * ui_settings_i2c_devices_create(lv_obj_t * parent)
{
    lv_obj_t * content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_style_pad_all(content, 10, 0);
    lv_obj_set_style_pad_gap(content, 10, 0);

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

    lv_label_set_text_fmt(i2c_devices_label, "No scan running.");

    return content;
}

static void i2c_refresh_clicked(lv_event_t * e)
{
    lv_label_set_text_fmt(i2c_devices_label, "Scanning...");
    global_tplp_->ScanI2cBus(&update_i2c_list);
}

static void update_i2c_list(const I2cScanResult& result)
{
    std::ostringstream txt;
    txt << "Status: " << result.status << "\n"
        << "Detected " << result.addresses.size() << " device(s)\n"
        << std::hex << std::setw(2) << std::setfill('0');
    for (uint8_t addr : result.addresses) {
        txt << "    0x" << (int)addr << "\n";
    }

    lv_label_set_text(i2c_devices_label, txt.str().c_str());
}
