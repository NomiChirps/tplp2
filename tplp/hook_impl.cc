// Implementation of various libraries' hooks for various things.

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/platform.h"
#include "pico/time.h"

extern "C" {
// FreeRTOS assertion failure handler
void FreeRTOS_AssertionFailed(const char *const file, unsigned long line) {
  panic("Assertion failed at: %s:%lu\n", file, line);
}

void FreeRTOS_ConfigureTimeForRunTimeStats() {
  // nothing to do; pico bootloader already starts the relevant timers.
}

unsigned long FreeRTOS_GetRunTimeCounterValue() {
  // Divide by 16 just to get smaller numbers so they fit better visually in the
  // summary table.
  return to_us_since_boot(get_absolute_time()) >> 4;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
  panic("Stack overflow in task: %s\n", name);
}

void lvgl_assertion_failed() { panic("lvgl assertion failed"); }

}  // extern "C"