#include "tplp/HX8357/lvgl_driver.h"

#include "hardware/sync.h"
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"
#include "tplp/graphics/lvgl_mutex.h"

namespace tplp {
namespace {
struct BlitInfo {
  TaskHandle_t lv_timer_task;
  TaskHandle_t blit_task;
  HX8357* display;
  lv_disp_drv_t* driver;
  lv_area_t area;
  const lv_color_t* bitmap;

  SemaphoreHandle_t blit_done_flag_;
};

std::ostream& operator<<(std::ostream& s, const lv_area_t& a) {
  return s << '(' << a.x1 << ',' << a.y1 << ")->(" << a.x2 << ',' << a.y2
           << ')';
}

static void flush_cb_impl(lv_disp_drv_t* driver, const lv_area_t* area,
                          lv_color_t* bitmap) {
  BlitInfo* blit_info = static_cast<BlitInfo*>(driver->user_data);
  // area needs to be copied because LVGL doesn't keep the pointer valid.
  VLOG(1) << "flush_cb_impl area = " << *area << ", bitmap = " << bitmap
          << ", bitmap[0] = " << *reinterpret_cast<const int16_t*>(&bitmap[0]);
  blit_info->area = *area;
  blit_info->bitmap = bitmap;
  xTaskNotifyGive(blit_info->blit_task);
}

static void wait_cb_impl(lv_disp_drv_t* driver) {
  CHECK_EQ(xTaskGetCurrentTaskHandle(),
           static_cast<BlitInfo*>(driver->user_data)->lv_timer_task);
  VLOG(1) << "wait_cb";
  xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
}

// Blitting is split off into a different task because SpiManager's transaction
// interface requires the calling task to wait for the transaction to complete,
// and we don't want to block flush_cb_impl. I mean, we *could* block it, but
// because it's run from the main LVGL timer task, LVGL wouldn't be able to do
// any more work until the blit finished, defeating the purpose of
// double-buffering.
static void BlitTask(void* param) {
  LOG(INFO) << "BlitTask started";
  BlitInfo* blit_info = static_cast<BlitInfo*>(param);
  for (;;) {
    VLOG(1) << "BlitTask waiting";
    ulTaskNotifyTake(true, portMAX_DELAY);
    // ARM docs claim that a memory barrier is not needed here because Cortex-M
    // does not reorder memory transactions. So we're ok to read blit_info.
    VLOG(1) << "BlitTask area = " << blit_info->area
            << ", bitmap = " << blit_info->bitmap << ", bitmap[0] = "
            << *reinterpret_cast<const int16_t*>(&blit_info->bitmap[0]);
    blit_info->display->Blit(
        reinterpret_cast<const uint16_t*>(blit_info->bitmap),
        blit_info->area.x1, blit_info->area.y1, blit_info->area.x2,
        blit_info->area.y2);
    // Cannot and should not use LvglLock for this call, because the lvgl
    // refresh code will in some cases wait (in the timer handler task) for a
    // flush to complete.
    lv_disp_flush_ready(blit_info->driver);
    // Wake up the lv timer task if it was waiting.
    xTaskNotify(blit_info->lv_timer_task, 0, eNoAction);
  }
}

}  // namespace

lv_disp_t* RegisterDisplayDriver_HX8357(HX8357* display,
                                        TaskHandle_t lv_timer_task) {
  LvglMutex mutex;
  // LVGL docs recommend 1/10 screen size as the maximum useful drawbuf size.
  // However, we're tight on memory already, so let's make it 1/20.
  const int kBufPixelCount = display->width() * display->height() / 20;
  const int kBufSize = kBufPixelCount * sizeof(lv_color_t);
  lv_disp_draw_buf_t* draw_buf = CHECK_NOTNULL(new lv_disp_draw_buf_t);
  VLOG(1) << "Allocating " << kBufSize << " bytes for buf1";
  lv_color_t* buf1 = CHECK_NOTNULL(new lv_color_t[kBufSize]);
  VLOG(1) << "Allocating " << kBufSize << " bytes for buf2";
  lv_color_t* buf2 = CHECK_NOTNULL(new lv_color_t[kBufSize]);
  lv_disp_draw_buf_init(draw_buf, buf1, buf2, kBufPixelCount);

  BlitInfo* blit_info = CHECK_NOTNULL(new BlitInfo);
  blit_info->lv_timer_task = lv_timer_task;
  CHECK(xTaskCreate(&BlitTask, "HX8357", TaskStacks::kHX8357, blit_info,
                    TaskPriorities::kHX8357, &blit_info->blit_task));

  lv_disp_drv_t* driver = CHECK_NOTNULL(new lv_disp_drv_t);
  lv_disp_drv_init(driver);
  driver->draw_buf = draw_buf;
  driver->hor_res = display->width();
  driver->ver_res = display->height();
  driver->flush_cb = &flush_cb_impl;
  driver->antialiasing = true;
  driver->sw_rotate = false;
  driver->full_refresh = false;
  driver->direct_mode = false;
  driver->user_data = blit_info;
  driver->wait_cb = &wait_cb_impl;

  blit_info->display = display;
  blit_info->driver = driver;
  return lv_disp_drv_register(driver);
}

}  // namespace tplp