#ifndef TPLP_LOGGING_H_
#define TPLP_LOGGING_H_

#include <chrono>
#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/mutex.h"
#include "pico/runtime.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "tplp/time.h"

namespace tplp {
namespace detail {
mutex_t* GetDebugLogMutex();
}  // namespace detail

// FIXME: for god's sake just make this a macro
template <typename... Params>
struct DebugLog {
  explicit DebugLog(Params&&... params,
                    const std::experimental::source_location& loc =
                        std::experimental::source_location::current()) {
    uint64_t timestamp = to_us_since_boot(get_absolute_time());
    mutex_enter_blocking(detail::GetDebugLogMutex());
    bool scheduler = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
    const char* task_name = "SCHEDULER_NOT_STARTED";
    if (scheduler) {
      task_name = pcTaskGetName(nullptr);
      if (!task_name) {
        task_name = "NOT_A_TASK";
      }
    }
    printf("I %llu.%06llu c%d %s %s:%lu] ", timestamp / 1000000,
           timestamp % 1000000, get_core_num(), task_name, loc.file_name(),
           loc.line());
    fflush(stdout);
    printf(std::forward<Params>(params)...);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    mutex_exit(detail::GetDebugLogMutex());
    // TODO: figure out why these fflush calls are necessary
    //       without them we get all the newlines printed FIRST (???)
    // pico stdio_usb has no explicit flush
  }
};

template <typename... Params>
DebugLog(Params&&...) -> DebugLog<Params...>;

// TODO: use a proper logging/assert framework...
// TODO: move body to .c file
#define tplp_assert(x)                                                  \
  do {                                                                  \
    if (!(x)) {                                                         \
      printf("[%s:%d] Assertion failed: %s\n", __FILE__, __LINE__, #x); \
      stdio_flush();                                                    \
      panic("tplp_assert failed");                                      \
    }                                                                   \
  } while (0)

#define tplp_assert_notnull(x)                                   \
  do {                                                           \
    if (!(x)) {                                                  \
      printf("[%s:%d] %s is nullptr\n", __FILE__, __LINE__, #x); \
      stdio_flush();                                             \
      panic("tplp_assert_notnull failed");                       \
    }                                                            \
  } while (0)

}  // namespace tplp

#endif  // TPLP_LOGGING_H_