#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "picolog/hal/hal.h"

namespace picolog {
namespace hal {

uint64_t get_us_since_boot() {
  timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);
  return tp.tv_sec * 1000000 + tp.tv_nsec / 1000;
}

void panic(const char* message) {
  fputs(message, stderr);
  exit(1);
}

}  // namespace hal
}  // namespace picolog