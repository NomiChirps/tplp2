#include "tplp/logging.h"

#include "pico/mutex.h"

namespace tplp {
namespace {
auto_init_mutex(global_debug_log_mutex_);
}  // namespace
namespace detail {
mutex_t* GetDebugLogMutex() { return &global_debug_log_mutex_; }
}  // namespace detail

}  // namespace tplp