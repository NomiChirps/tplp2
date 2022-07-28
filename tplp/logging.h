#ifndef TPLP_LOGGING_H_
#define TPLP_LOGGING_H_

#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "fmt/format.h"
#include "pico/mutex.h"
#include "pico/stdio.h"

namespace tplp {
namespace detail {
mutex_t* GetDebugLogMutex();
}  // namespace detail

template <typename... Params>
struct DebugLog {
  explicit DebugLog(Params&&... params,
                    const std::experimental::source_location& loc =
                        std::experimental::source_location::current()) {
    // TODO: add timestamp
    // TODO: add current task name
    // Note that fmt::print() is very much thread-unsafe, on top of us wanting
    // to serialize log lines in any case.
    std::string msg = fmt::format(std::forward<Params>(params)...);
    mutex_t* mutex = detail::GetDebugLogMutex();
    mutex_enter_blocking(mutex);
    // TODO: use https://fmt.dev/latest/api.html#format-string-compilation
    fmt::print("I c{3} {1}:{2}] {0}\n", msg, loc.file_name(), loc.line(),
               get_core_num());
    stdio_flush();
    mutex_exit(mutex);
  }
};

template <typename... Params>
DebugLog(Params&&...) -> DebugLog<Params...>;

}  // namespace tplp

#endif  // TPLP_LOGGING_H_