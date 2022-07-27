#include "tplp/graphics/lvgl_mutex.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

namespace tplp {
namespace {

SemaphoreHandle_t global_lvgl_mutex = 0;

}  // namespace

void LvglLock::InitOnce() {
  if (!global_lvgl_mutex) {
    global_lvgl_mutex = xSemaphoreCreateRecursiveMutex();
  }
}

LvglLock::LvglLock() {
  xSemaphoreTakeRecursive(global_lvgl_mutex, portMAX_DELAY);
}

LvglLock::~LvglLock() { xSemaphoreGiveRecursive(global_lvgl_mutex); }

}  // namespace tplp