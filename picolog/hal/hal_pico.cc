#include "pico/time.h"
#include "picolog/hal/hal.h"

namespace picolog {
namespace hal {

uint64_t get_us_since_boot() { return to_us_since_boot(get_absolute_time()); }

void panic(const char* message) { ::panic(message); }

}  // namespace hal
}  // namespace picolog