#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Use Pico SDK ISR handlers
// TODO: learn what this is for and why
#define vPortSVCHandler isr_svcall
#define xPortPendSVHandler isr_pendsv
#define xPortSysTickHandler isr_systick

// This is ignored because we're using heap_3, which just wraps malloc/free.
#define configTOTAL_HEAP_SIZE 0

#define configUSE_PREEMPTION 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE 0
#define configCPU_CLOCK_HZ 133000000
#define configTICK_RATE_HZ 100
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 128
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 3
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 0
#define configQUEUE_REGISTRY_SIZE 10
#define configUSE_QUEUE_SETS 0
#define configUSE_TIME_SLICING 0
// I think this means cstdlib functions are safe(r) in ISRs?
#define configUSE_NEWLIB_REENTRANT 1
#define configENABLE_BACKWARD_COMPATIBILITY 0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5
#define configSTACK_DEPTH_TYPE uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t

// Memory allocation related definitions.
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configAPPLICATION_ALLOCATED_HEAP 0

// Hook function related definitions.
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
// TODO(for release): disable stack overflow checking; it has overhead
#define configCHECK_FOR_STACK_OVERFLOW 1
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

// Run time and task stats gathering related definitions.
// TODO(for release): disable runtime stats
#define configGENERATE_RUN_TIME_STATS 1
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
extern void FreeRTOS_ConfigureTimeForRunTimeStats();
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS \
  FreeRTOS_ConfigureTimeForRunTimeStats
extern unsigned long FreeRTOS_GetRunTimeCounterValue();
#define portGET_RUN_TIME_COUNTER_VALUE FreeRTOS_GetRunTimeCounterValue

// Co-routine related definitions.
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES 1

// Software timer related definitions.
#define configUSE_TIMERS 1
// TODO: what should the timer task's priority be? high or low?
#define configTIMER_TASK_PRIORITY 3
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH configMINIMAL_STACK_SIZE

// Define to trap errors during development.
// TODO(for release): disable configASSERT
extern void FreeRTOS_AssertionFailed(const char* const file,
                                     unsigned long line);
#define configASSERT(x)                           \
  if ((x) == 0) {                                 \
    FreeRTOS_AssertionFailed(__FILE__, __LINE__); \
  }

// Optional functions - most linkers will remove unused functions anyway.
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xResumeFromISR 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 0
#define INCLUDE_xTaskGetIdleTaskHandle 0
#define INCLUDE_eTaskGetState 0
#define INCLUDE_xEventGroupSetBitFromISR 1
#define INCLUDE_xTimerPendFunctionCall 1
#define INCLUDE_xTaskAbortDelay 0
#define INCLUDE_xTaskGetHandle 0
#define INCLUDE_xTaskResumeFromISR 1

// A header file that defines trace macro can be included here.

#endif /* FREERTOS_CONFIG_H */