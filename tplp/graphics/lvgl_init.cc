#include "tplp/graphics/lvgl_init.h"

#include <functional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl/lvgl.h"
#include "picolog/picolog.h"
#include "tplp/HX8357/lvgl_driver.h"
#include "tplp/SharpLCD/lvgl_driver.h"
#include "tplp/TSC2007/lvgl_driver.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/graphics/mouse_cursor_icon.h"
#include "tplp/rtos_util.h"

namespace tplp {
namespace {

void LvglTimerHandlerTask(void*) {
  LOG(INFO) << "LVGL timer handler task pending.";
  // Wait to begin until init is finished.
  ulTaskNotifyTake(true, portMAX_DELAY);
  LOG(INFO) << "LVGL timer handler task started.";
  for (;;) {
    {
      LvglMutex lock;
      VLOG(2) << "calling lv_timer_handler";
      lv_timer_handler();
    }
    // This timer task should be a very low priority. LVGL will happily soak up
    // all the CPU time we can throw at it and put it into making the UI more
    // reponsive; but this should obviously not be at the expense of anything
    // more important, which is almost anything.
    taskYIELD();
  }
}

void lvgl_print_cb_impl(const char* msg) {
  // TODO: improve formatting
  // (Hackily) strip newline
  int len = strlen(msg);
  if (len && msg[len - 1] == '\n') {
    const_cast<char*>(msg)[len - 1] = '\0';
  }
  LOG(WARNING) << msg;
}

TaskHandle_t CreateTimerHandlerTask(int priority, int stack_depth) {
  TaskHandle_t result;
  CHECK(xTaskCreate(&LvglTimerHandlerTask, "LVGL", stack_depth, nullptr,
                    priority, &result));
  return result;
}

void FinishDisplaySetup(lv_disp_t* display) {
  lv_theme_t* theme = lv_theme_default_init(
      display, lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
  lv_disp_set_theme(display, theme);
}

}  // namespace

struct LvglInit::Objects {
  TaskHandle_t timer_task = nullptr;

  lv_indev_t* touchscreen = nullptr;
};

LvglInit::LvglInit() : stuff_(CHECK_NOTNULL(new Objects)) {}

void LvglInit::BaseInit(int priority, int stack_depth) {
  LvglMutex::InitOnce();
  LvglMutex lock;
  // LVGL documentation says:
  // 1. Call lv_init().
  // 2. Initialize your drivers.
  // 3. Register the display and input devices drivers in LVGL.
  // 4. Call lv_tick_inc(x) every x milliseconds in an interrupt to report the
  //    elapsed time to LVGL (or set LV_TICK_CUSTOM in lv_conf.h).
  // 5. Call lv_timer_handler() every few milliseconds to handle LVGL related
  //    tasks.
  lv_log_register_print_cb(&lvgl_print_cb_impl);
  lv_init();

  // Task will wait until we xTaskNotifyGive() it.
  stuff_->timer_task = CreateTimerHandlerTask(priority, stack_depth);
  // No need for a lv_tick_inc() interrupt because we're using LV_TICK_CUSTOM.
  static_assert(LV_TICK_CUSTOM);
}

void LvglInit::SetDisplay(SharpLCD* raw_display, int priority,
                          int stack_depth) {
  lv_disp_t* display =
      RegisterDisplayDriver_SharpLCD(raw_display, priority, stack_depth);
  FinishDisplaySetup(display);
}

void LvglInit::SetDisplay(HX8357* raw_display, int priority, int stack_depth) {
  CHECK(stuff_->timer_task) << "call BaseInit() first";
  lv_disp_t* display = RegisterDisplayDriver_HX8357(
      priority, stack_depth, raw_display, stuff_->timer_task);
  FinishDisplaySetup(display);
}

void LvglInit::AddTouchscreen(TSC2007* raw_touchscreen) {
  stuff_->touchscreen = RegisterInputDevice_TSC2007(raw_touchscreen);
}

void LvglInit::Start() {
  if (stuff_->touchscreen) {
    lv_obj_t* icon = lv_img_create(lv_scr_act());
    lv_img_set_src(icon, &mouse_cursor_icon);
    lv_indev_set_cursor(stuff_->touchscreen, icon);
  }

  xTaskNotifyGive(stuff_->timer_task);
}

}  // namespace tplp
