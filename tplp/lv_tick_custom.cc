#include "tplp/lv_tick_custom.h"

#include "pico/time.h"

extern "C" {
uint32_t lv_tick_custom_impl() { return to_ms_since_boot(get_absolute_time()); }
}