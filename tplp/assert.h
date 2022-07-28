#ifndef TPLP_ASSERT_H_
#define TPLP_ASSERT_H_

#include "pico/runtime.h"

// TODO: use a proper logging/assert framework...
// TODO: move body to .c file
#define tplp_assert(x)                                                  \
  do {                                                                  \
    if (!(x)) {                                                         \
      printf("[%s:%d] Assertion failed: %s\n", __FILE__, __LINE__, #x); \
      stdio_flush();                                                    \
      panic("Assertion failed");                                        \
    }                                                                   \
  } while (0)

#define tplp_assert_notnull(x)                                   \
  do {                                                           \
    if (!(x)) {                                                  \
      printf("[%s:%d] %s is nullptr\n", __FILE__, __LINE__, #x); \
      stdio_flush();                                             \
      panic("Assertion failed");                                 \
    }                                                            \
  } while (0)

#endif  // TPLP_ASSERT_H_