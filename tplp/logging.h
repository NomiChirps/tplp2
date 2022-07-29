#ifndef TPLP_LOGGING_H_
#define TPLP_LOGGING_H_

#include <chrono>
#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "pico/stdio.h"
#include "tplp/time.h"

namespace tplp {
namespace detail {
constexpr int kDebugLogBufSize = 2048;
extern SemaphoreHandle_t global_debug_log_mutex_;
extern char global_debug_log_buf_[kDebugLogBufSize];
}  // namespace detail

// FIXME: for god's sake just make this a macro
template <typename... Params>
struct DebugLog {
  explicit DebugLog(Params&&... params,
                    const std::experimental::source_location& loc =
                        std::experimental::source_location::current()) {
    // TODO: add timestamp
    // TODO: add current task name
    // Note that fmt::print() is very much thread-unsafe, on top of us wanting
    // to serialize log lines in any case.
    bool need_lock = xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
    if (need_lock) {
      if (!detail::global_debug_log_mutex_) {
        panic("global_debug_log_mutex_ == 0");
      }
      xSemaphoreTake(detail::global_debug_log_mutex_,
                     as_ticks(std::chrono::milliseconds(100)));
    }
    snprintf(detail::global_debug_log_buf_, detail::kDebugLogBufSize,
             std::forward<Params>(params)...);
    printf("Ic%d %s:%lu] %s", get_core_num(), loc.file_name(), loc.line(),
           detail::global_debug_log_buf_);
    fflush(stdout);
    putchar('\n');
    stdio_flush();
    if (need_lock) {
      xSemaphoreGive(detail::global_debug_log_mutex_);
    }
  }
};

template <typename... Params>
DebugLog(Params&&...) -> DebugLog<Params...>;

// Call once from main().
void DebugLogStaticInit();

}  // namespace tplp

#endif  // TPLP_LOGGING_H_