#ifndef TPLP_LV_TICK_CUSTOM_H_
#define TPLP_LV_TICK_CUSTOM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// Returns the current time in milliseconds.
uint32_t lv_tick_custom_impl();
#ifdef __cplusplus
}
#endif

#endif  // TPLP_LV_TICK_CUSTOM_H_