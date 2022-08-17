#include "picolog/hal/backtrace.h"

namespace picolog {
namespace hal {

int TakeBacktrace(backtrace_frame_t* buffer, int size) {
  if (size) {
    buffer[0] = {
        .ip = nullptr,
        .name = "<backtrace not supported>",
    };
    return 1;
  }
  return 0;
}

}  // namespace hal
}  // namespace picolog