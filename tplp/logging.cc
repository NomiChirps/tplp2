#include "tplp/logging.h"

#include "pico/mutex.h"

namespace tplp {
namespace detail {
namespace {
auto_init_mutex(global_debug_log_mutex_);
}  // namespace
mutex_t* GetDebugLogMutex() { return &global_debug_log_mutex_; }
}  // namespace detail
}  // namespace tplp