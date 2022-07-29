#include "tplp/logging.h"

namespace tplp {
namespace detail {
SemaphoreHandle_t global_debug_log_mutex_ = 0;
char global_debug_log_buf_[kDebugLogBufSize];
}  // namespace detail

void DebugLogStaticInit() {
  static StaticQueue_t global_debug_log_mutex_buf_;
  detail::global_debug_log_mutex_ =
      xSemaphoreCreateMutexStatic(&global_debug_log_mutex_buf_);
}

}  // namespace tplp