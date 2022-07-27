#include "tplp/graphics/graphics.h"

#include <chrono>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl/lvgl.h"
#include "tplp/SharpLCD/lvgl_driver.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"

using std::chrono_literals::operator""ms;

namespace tplp {
namespace {

void LvglTimerHandlerTask(void*) {
  DebugLog("LVGL timer handler task started.");
  for (;;) {
    {
      LvglLock lock;
      lv_timer_handler();
    }
    // TODO: should this be shorter?
    vTaskDelay(as_ticks(100ms));
  }
}

}  // namespace

void InitLvgl(SharpLCD* display) {
  LvglLock::InitOnce();
  LvglLock lock;
  // LVGL documentation says:
  // 1. Call lv_init().
  // 2. Initialize your drivers.
  // 3. Register the display and input devices drivers in LVGL.
  // 4. Call lv_tick_inc(x) every x milliseconds in an interrupt to report the
  //    elapsed time to LVGL (or set LV_TICK_CUSTOM in lv_conf.h).
  // 5. Call lv_timer_handler() every few milliseconds to handle LVGL related
  //    tasks.
  lv_init();
  InitAndRegisterDisplayDriver(display);

  lv_theme_mono_init(nullptr, 0, lv_font_default());
  // TODO: register input device drivers

  // No need for a lv_tick_inc() interrupt because we're using LV_TICK_CUSTOM.
  static_assert(LV_TICK_CUSTOM);
  // TODO: determine necessary stack depth
  xTaskCreate(&LvglTimerHandlerTask, "lv_timer_handler", 1024, nullptr,
              TaskPriorities::kLvglTimerHandler, nullptr);
}

void RunLvglDemo(void*) {
  DebugLog("RunLvglDemo task started.");
  lv_obj_t* label1;
  {
    LvglLock lock;
    label1 = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_label_set_text(label1,
                      "Hello, world! blah, blah, blah, blah, blah, blah, blah");
    lv_obj_set_width(label1, 120); /*Set smaller width to make the lines wrap*/
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_t* label2 = lv_label_create(lv_scr_act());
    // lv_label_set_long_mode(label2,
    //                        LV_LABEL_LONG_SCROLL_CIRCULAR); /*Circular
    //                        scroll*/
    // lv_obj_set_width(label2, 150);
    // lv_label_set_text(label2, "It is a circularly scrolling text. ");
    // lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);
  }
  for (;;) {
    vTaskDelay(as_ticks(400ms));
    {
      LvglLock lock;
      lv_obj_align(label1, LV_ALIGN_CENTER, -10, 0);
    }
    vTaskDelay(as_ticks(400ms));
    {
      LvglLock lock;
      lv_obj_align(label1, LV_ALIGN_CENTER, 10, 0);
    }
  }
}

}  // namespace tplp
