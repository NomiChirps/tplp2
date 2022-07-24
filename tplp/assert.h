#ifndef TPLP_ASSERT_H_
#define TPLP_ASSERT_H_

#include "pico/runtime.h"

#define tplp_assert(x)                                                    \
  do {                                                                    \
    if (!(x)) panic("[" __FILE__ ":%d] Assertion failed: " #x, __LINE__); \
  } while (0)

#endif  // TPLP_ASSERT_H_