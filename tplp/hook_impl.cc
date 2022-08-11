// Implementation of various libraries' hooks for various things.

#include <cstdio>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "picolog/picolog.h"

extern "C" {
// FreeRTOS assertion failure handler
void FreeRTOS_AssertionFailed(const char* const file, unsigned long line) {
  LOG(FATAL) << "FreeRTOS assertion failed at " << file << ":" << line;
}

void FreeRTOS_ConfigureTimeForRunTimeStats() {
  // nothing to do; pico bootloader already starts the relevant timers.
}

unsigned long FreeRTOS_GetRunTimeCounterValue() {
  return to_us_since_boot(get_absolute_time());
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
  LOG(FATAL) << "Stack overflow in task " << name;
}

void lvgl_assertion_failed() {
  LOG(FATAL) << "lvgl assertion failed (no further information)";
}

// configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so
// the application must provide an implementation of
// vApplicationGetTimerTaskMemory() to provide the memory that is used by the
// Timer service task.
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) {
  // If the buffers to be provided to the Timer task are declared inside this
  // function then they must be declared static - otherwise they will be
  // allocated on the stack and so not exists after this function exits.
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  // Pass out a pointer to the StaticTask_t structure in which the Timer
  // task's state will be stored.
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  // Pass out the array that will be used as the Timer task's stack.
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  // Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
  // Note that, as the array is necessarily of type StackType_t,
  // configTIMER_TASK_STACK_DEPTH is specified in words, not bytes.
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"