#include "tplp/graphics/lvgl_mutex.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

namespace tplp {
namespace {

static SemaphoreHandle_t global_lvgl_mutex_ = 0;

}  // namespace

void LvglLock::InitOnce() {
  if (!global_lvgl_mutex_) {
    global_lvgl_mutex_ = xSemaphoreCreateRecursiveMutex();
  }
}

LvglLock::LvglLock() {
  xSemaphoreTakeRecursive(global_lvgl_mutex_, portMAX_DELAY);
}

LvglLock::~LvglLock() { xSemaphoreGiveRecursive(global_lvgl_mutex_); }

}  // namespace tplp