#include "tplp/SharpLCD/lvgl_driver.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/task.h"
#include "picolog/picolog.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/tplp_config.h"


namespace tplp {
namespace {

struct DriverUserData {
  SharpLCD* display;
  // The framebuffer currently being drawn on.
  SharpLCD::FrameBuffer fb_front;
  // The framebuffer currently being sent to the display.
  SharpLCD::FrameBuffer fb_back;
  // 0 while fb_back is being transmitted. 1 while the display is idle.
  SemaphoreHandle_t display_ready;
  // Task handle of DisplayRefreshTask() below.
  TaskHandle_t refresh_task;
};

void flush_cb_impl(lv_disp_drv_t* driver, const lv_area_t* area,
                   lv_color_t* bitmap) {
  DriverUserData* user_data = static_cast<DriverUserData*>(driver->user_data);
  user_data->fb_front.BitBlit(reinterpret_cast<uint8_t*>(bitmap),
                              area->x2 - area->x1 + 1, area->y2 - area->y1 + 1,
                              area->x1, area->y1);
  // If the previous frame is done being transmitted, swap buffers and trigger a
  // new transmission.
  if (xSemaphoreTake(user_data->display_ready, /*ticks_to_wait=*/0)) {
    std::swap(user_data->fb_front, user_data->fb_back);
    xTaskNotify(user_data->refresh_task,
                reinterpret_cast<uint32_t>(&user_data->fb_back),
                eSetValueWithOverwrite);
  }

  // Ready for the next update from LVGL.
  {
    LvglLock mutex;
    lv_disp_flush_ready(driver);
  }
}

void set_px_cb_impl(lv_disp_drv_t* disp_drv, uint8_t* buf, lv_coord_t buf_w,
                    lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t) {
  buf += y * buf_w / 8 + x / 8;
  if (lv_color_brightness(color) > 128) {
    (*buf) |= (0x80 >> (x % 8));
  } else {
    (*buf) &= ~(0x80 >> (x % 8));
  }
}

struct DisplayRefreshTaskData {
  SharpLCD* display;
  SemaphoreHandle_t display_ready;
};

void DisplayRefreshTask(void* param) {
  LOG(INFO) << "Display refresh task started.";
  auto data = static_cast<DisplayRefreshTaskData*>(param);
  // Allegedly, the display sometimes needs to be cleared twice on startup.
  data->display->Clear();
  data->display->Clear();
  // Ready for the first frame.
  xSemaphoreGive(data->display_ready);
  for (;;) {
    // Wait for a frame to be ready.
    auto* fb = reinterpret_cast<SharpLCD::FrameBuffer*>(
        ulTaskNotifyTake(true, portMAX_DELAY));
    data->display->DrawFrameBuffer(*fb, [data]() {
      // Indicate that we're ready for the next frame.
      xSemaphoreGive(data->display_ready);
    });
  }
}

}  // namespace

lv_disp_t* RegisterDisplayDriver_SharpLCD(SharpLCD* display) {
  // Doesn't actually need to be screen-sized, but we get better "vsync" if it
  // is, since we don't need to coalesce multiple flush_cb calls to update the
  // entire framebuffer. It also ends up being way more efficient.
  const uint32_t kBufPixelCount = SharpLCD::width() * SharpLCD::height();
  // Packed into bits.
  const uint32_t kBufSize = kBufPixelCount / 8;

  LvglLock mutex;

  lv_disp_draw_buf_t* draw_buf = new lv_disp_draw_buf_t;
  lv_color_t* buf1 = new lv_color_t[kBufSize];
  // No need for LVGL's double buffering; we do our own.
  lv_disp_draw_buf_init(draw_buf, buf1, /*buf2=*/nullptr, kBufPixelCount);

  DisplayRefreshTaskData* task_param = new DisplayRefreshTaskData{
      .display = display,
      .display_ready = xSemaphoreCreateBinary(),
  };
  TaskHandle_t refresh_task;
  CHECK(xTaskCreate(&DisplayRefreshTask, "lvgl_disp_drv", TaskStacks::kDefault,
                    task_param, TaskPriorities::kLvglDisplayDriver,
                    &refresh_task));

  lv_disp_drv_t* driver = new lv_disp_drv_t;
  lv_disp_drv_init(driver);
  driver->draw_buf = draw_buf;
  driver->hor_res = SharpLCD::width();
  driver->ver_res = SharpLCD::height();
  driver->flush_cb = &flush_cb_impl;
  driver->set_px_cb = &set_px_cb_impl;
  driver->full_refresh = false;
  driver->direct_mode = false;
  driver->user_data = new DriverUserData{
      .display = display,
      .fb_front = SharpLCD::AllocateNewFrameBuffer(),
      .fb_back = SharpLCD::AllocateNewFrameBuffer(),
      .display_ready = task_param->display_ready,
      .refresh_task = refresh_task,
  };
  return lv_disp_drv_register(driver);
}

}  // namespace tplp
