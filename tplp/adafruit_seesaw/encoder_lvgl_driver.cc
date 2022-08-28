#include "tplp/adafruit_seesaw/encoder_lvgl_driver.h"

#include "picolog/picolog.h"

namespace tplp {
namespace {
void read_cb_impl(lv_indev_drv_t* driver, lv_indev_data_t* data) {
  SeesawEncoder* encoder = static_cast<SeesawEncoder*>(driver->user_data);
  auto maybe_delta = encoder->GetDelta();
  if (!maybe_delta.ok()) {
    LOG(ERROR) << "Failed to read encoder: " << maybe_delta.status();
    return;
  }
  VLOG(1) << "delta = " << *maybe_delta;
  // TODO: the button!!!
  data->enc_diff = *maybe_delta;
  data->state = LV_INDEV_STATE_RELEASED;
}
}  // namespace

lv_indev_t* RegisterInputDevice_SeesawEncoder(SeesawEncoder* encoder) {
  // TODO: consider using the interrupt for presses
  lv_indev_drv_t* indev_drv = CHECK_NOTNULL(new lv_indev_drv_t);
  lv_indev_drv_init(indev_drv);
  indev_drv->type = LV_INDEV_TYPE_ENCODER;
  indev_drv->read_cb = &read_cb_impl;
  indev_drv->user_data = encoder;
  return lv_indev_drv_register(indev_drv);
}

}  // namespace tplp