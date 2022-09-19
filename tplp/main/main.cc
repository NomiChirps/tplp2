#include <cstdio>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tplp/config/hw.h"
#include "tplp/config/tasks.h"
#include "tplp/main/startup.h"

#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif
#if LIB_PICO_STDIO_UART
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdio_uart.h"
#endif

int main() {
#if LIB_PICO_STDIO_USB
  stdio_usb_init();
#endif
#if LIB_PICO_STDIO_UART
  stdio_uart_init_full(uart0, 115200, tplp::Pins::UART0_TX, -1);
  // Invert UART signal to be compatible with RS232.
  gpio_set_outover(tplp::Pins::UART0_TX, GPIO_OVERRIDE_INVERT);
  busy_wait_ms(100);
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
