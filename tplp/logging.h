#ifndef TPLP_LOGGING_H_
#define TPLP_LOGGING_H_

#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "fmt/format.h"
#include "pico/stdio.h"

namespace tplp {

template <typename... Params>
struct DebugLog {
  explicit DebugLog(Params&&... params,
                    const std::experimental::source_location& loc =
                        std::experimental::source_location::current()) {
    // TODO: add timestamp
    // TODO: add current task name
    fmt::print("I {1}:{2}] {0}\n", fmt::format(std::forward<Params>(params)...),
               loc.file_name(), loc.line());
    stdio_flush();
  }
};

template <typename... Params>
DebugLog(Params&&...) -> DebugLog<Params...>;

}  // namespace tplp

#endif  // TPLP_LOGGING_H_