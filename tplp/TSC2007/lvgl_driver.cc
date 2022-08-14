#include "tplp/TSC2007/lvgl_driver.h"

#include "picolog/picolog.h"
#include "tplp/TSC2007/TSC2007.h"

namespace tplp {
namespace {
void read_cb_impl(lv_indev_drv_t* driver, lv_indev_data_t* data) {
  lv_coord_t hor_res = lv_disp_get_physical_hor_res(driver->disp);
  lv_coord_t ver_res = lv_disp_get_physical_ver_res(driver->disp);
  TSC2007* touchscreen = static_cast<TSC2007*>(driver->user_data);
  int16_t x, y, z1, z2;
  util::Status status = touchscreen->ReadPosition(&x, &y, &z1, &z2);
  if (!status.ok()) {
    // snap back to the middle of the screen to hopefully make it obvious something wrong happened
    data->point.x = hor_res/2;
    data->point.y = ver_res/2;
    data->state = LV_INDEV_STATE_RELEASED;
    LOG(ERROR) << "Failed to read touchscreen: " << status;
    return;
  }
  const int pressure_threshold = 20;
  if (z1 < pressure_threshold || z2 < pressure_threshold) {
    // Don't change x and y; those values are garbage when pressure is zero.
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  } else {
    data->state = LV_INDEV_STATE_PRESSED;
  }

  // Transform determined empirically.
  LOG(INFO) << x << "," << y << " " << z1 << ":" << z2;
  // Turns out also that x is vertical and y is horizontal.
  // FIXME: we need to get this from the display's rotation setting
  const int min_x = 300;
  const int max_x = 3700;
  const int min_y = 200;
  const int max_y = 3860;
  x = ((x - min_x)*ver_res)/(max_x - min_x);
  y = ((y - min_y)*hor_res)/(max_y - min_y);
  data->point.x = std::max<int16_t>(0, std::min<int16_t>(hor_res-1, y));
  data->point.y = std::max<int16_t>(0, std::min<int16_t>(ver_res-1, ver_res - x));

  // TODO: provide buffering
  // "By default, LVGL calls read_cb periodically. Because of this intermittent
  // polling there is a chance that some user gestures are missed. To solve this
  // you can write an event driven driver for your input device that buffers
  // measured data. In read_cb you can report the buffered data instead of
  // directly reading the input device. Setting the data->continue_reading flag
  // will tell LVGL there is more data to read and it should call read_cb
  // again."
}
}  // namespace

lv_indev_t* RegisterInputDevice_TSC2007(TSC2007* touchscreen) {
  lv_indev_drv_t* indev_drv = CHECK_NOTNULL(new lv_indev_drv_t);
  lv_indev_drv_init(indev_drv);
  indev_drv->type = LV_INDEV_TYPE_POINTER;
  indev_drv->read_cb = &read_cb_impl;
  indev_drv->user_data = touchscreen;
  return lv_indev_drv_register(indev_drv);
}

}  // namespace tplp