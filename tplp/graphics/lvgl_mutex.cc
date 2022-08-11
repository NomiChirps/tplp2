#include "tplp/graphics/lvgl_mutex.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "picolog/picolog.h"

namespace tplp {
namespace {

SemaphoreHandle_t global_lvgl_mutex_;

}  // namespace

LvglMutex::LvglMutex() {
  xSemaphoreTakeRecursive(global_lvgl_mutex_, portMAX_DELAY);
  // TODO: it'd be nice to have the file & line number of the caller
  VLOG(1) << "-> TAKE";
}

LvglMutex::~LvglMutex() {
  VLOG(1) << "<- GIVE";
  xSemaphoreGiveRecursive(global_lvgl_mutex_);
}

void LvglMutex::InitOnce() {
  global_lvgl_mutex_ = xSemaphoreCreateRecursiveMutex();
}

}  // namespace tplp