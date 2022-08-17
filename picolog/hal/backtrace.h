#ifndef PICOLOG_HAL_BACKTRACE_H_
#define PICOLOG_HAL_BACKTRACE_H_

namespace picolog {
namespace hal {

struct backtrace_frame_t {
  // Instruction pointer.
  const void* ip;
  // Name of the surrounding function. Never null. Does not need to be freed.
  // Guaranteed to be null-terminated. If this is "<unknown>" all the time, you
  // might need `-mpoke-function-name` (on ARM)
  const char* name;
};

// Returns the number of frames that were written to the buffer, up to a maximum
// of `buffer_length`. Note that the first frame is likely to be the
// TakeBacktrace() function itself.
int TakeBacktrace(backtrace_frame_t* buffer, int buffer_length);

}  // namespace hal
}  // namespace picolog

#endif  // PICOLOG_HAL_BACKTRACE_H_