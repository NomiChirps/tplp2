#ifndef PICOLOG_HAL_HAL_H_
#define PICOLOG_HAL_HAL_H_

#include <cstdint>

namespace picolog {
namespace hal {

uint64_t get_us_since_boot();
[[noreturn]] void panic(const char* message);

}  // namespace hal
}  // namespace picolog

#endif  // PICOLOG_HAL_HAL_H_