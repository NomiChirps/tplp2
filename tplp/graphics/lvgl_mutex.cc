#include "tplp/graphics/lvgl_mutex.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

namespace tplp {
namespace {

static SemaphoreHandle_t global_lvgl_mutex_;

}  // namespace

LvglLock::LvglLock() {
  xSemaphoreTakeRecursive(global_lvgl_mutex_, portMAX_DELAY);
}

LvglLock::~LvglLock() { xSemaphoreGiveRecursive(global_lvgl_mutex_); }

void LvglLock::InitOnce() {
  global_lvgl_mutex_ = xSemaphoreCreateRecursiveMutex();
}

}  // namespace tplp