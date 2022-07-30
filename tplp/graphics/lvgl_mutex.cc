#include "tplp/graphics/lvgl_mutex.h"

#include "pico/mutex.h"

namespace tplp {
namespace {

auto_init_recursive_mutex(global_lvgl_mutex_);

}  // namespace

LvglLock::LvglLock() { recursive_mutex_enter_blocking(&global_lvgl_mutex_); }

LvglLock::~LvglLock() { recursive_mutex_exit(&global_lvgl_mutex_); }

}  // namespace tplp