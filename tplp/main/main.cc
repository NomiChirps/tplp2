#include <cstdio>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/stdio_usb.h"
#include "tplp/config/tasks.h"
#include "tplp/main/startup.h"

int main() {
#if LIB_PICO_STDIO_USB
  stdio_usb_init();
#endif
#if LIB_PICO_STDIO_UART
  stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, Pins::UART_TX,
                       -1);
#endif

  // TODO: add a build timestamp and version line at bootup
  // printf("tplp2 built %s at %s", TPLP_BUILD_TIMESTAMP, TPLP_VERSION_ID);
  printf("tplp2 boot. TODO: add a build timestamp\n");

  xTaskCreate(&tplp::StartupTask, "STARTUP", tplp::TaskStacks::kStartup,
              nullptr, tplp::TaskPriorities::kStartup, nullptr);
  vTaskStartScheduler();
  panic("scheduler died :(");

  return 1;
}
