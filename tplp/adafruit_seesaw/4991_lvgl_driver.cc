#include "tplp/adafruit_seesaw/4991_lvgl_driver.h"

#include "picolog/picolog.h"

namespace tplp {
namespace {
void read_cb_impl(lv_indev_drv_t* driver, lv_indev_data_t* data) {
  Adafruit4991* encoder = static_cast<Adafruit4991*>(driver->user_data);
  auto maybe_position = encoder->GetPosition();
  auto maybe_delta = encoder->GetDelta();
  //auto maybe_switch = encoder->GetSwitchState();
  auto maybe_switch = util::StatusOr<bool>(0);
  if (!maybe_delta.ok()) {
    LOG(ERROR) << "Failed to read encoder delta: " << maybe_delta.status();
    return;
  }
  if (!maybe_switch.ok()) {
    LOG(ERROR) << "Failed to read encoder switch: " << maybe_switch.status();
    return;
  }
  if (!maybe_position.ok()) {
    LOG(ERROR) << "Failed to read encoder position: " << maybe_position.status();
    return;
  }
  VLOG(1) << "delta = " << *maybe_delta << " switch = " << *maybe_switch << " position = " << *maybe_position;
  data->enc_diff = *maybe_delta;
  data->state =
      *maybe_switch ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}
}  // namespace

lv_indev_t* RegisterInputDevice_Adafruit4991(Adafruit4991* encoder) {
  // TODO: consider using the interrupt for presses
  lv_indev_drv_t* indev_drv = CHECK_NOTNULL(new lv_indev_drv_t);
  lv_indev_drv_init(indev_drv);
  indev_drv->type = LV_INDEV_TYPE_ENCODER;
  indev_drv->read_cb = &read_cb_impl;
  indev_drv->user_data = encoder;
  return lv_indev_drv_register(indev_drv);
}

}  // namespace tplp