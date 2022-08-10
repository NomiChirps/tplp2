#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/stdlib.h"
#include "picolog/picolog.h"
#include "tplp/HX8357/HX8357.h"
#include "tplp/RuntimeStats.h"
#include "tplp/SpiManager.h"
#include "tplp/config/tplp_config.h"
#include "tplp/graphics/lvgl_init.h"
#include "tplp/time.h"
#include "tplp/types.h"
#include "tplp/ws2812.h"

using std::chrono_literals::operator""ms;

#define CHECK_OK(expr) CHECK_EQ((expr), SpiTransaction::Result::OK)

namespace tplp {

void led_task(void*) {
  LOG(INFO) << "led_task started.";
  const int LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  while (true) {
    gpio_put(LED_PIN, 1);
    vTaskDelay(as_ticks(200ms));
    gpio_put(LED_PIN, 0);
    vTaskDelay(as_ticks(200ms));
  }
}

void neopixel_task(void*) {
  // TODO: play with neopixel
  // ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq,
  // bool rgbw)
}

void tft_test_task(void* task_param) {
  HX8357D* display = static_cast<HX8357D*>(task_param);
  for (;;) {
    vTaskDelay(as_ticks(4'000ms));
    display->SelfTest();
  }
}

// TODO: move this to its own file.
void StartupTask(void*) {
  picolog::InitLogging();
  if (!xTaskCreate(&picolog::BackgroundTask, "LOGGER", TaskStacks::kLogging,
                   nullptr, TaskPriorities::kLogging, nullptr)) {
    panic("Failed to start log task");
  }
  LOG(INFO) << "Begin startup...";

  StartRuntimeStatsReportingTask(/*priority=*/1);
  CHECK(xTaskCreate(&led_task, "blinky", TaskStacks::kDefault, nullptr, 1,
                    nullptr));

  // TODO: increase frequency
  SpiManager* spi1_manager =
      SpiManager::Init(TaskPriorities::kSpiManager1, spi1, 500'000,
                       Pins::SPI1_SCLK, Pins::SPI1_MOSI, Pins::SPI1_MISO);
  LOG(INFO) << "SpiManager::Init() OK";

  HX8357* display = new HX8357D(spi1_manager, Pins::HX8357_CS, Pins::HX8357_DC);
  display->Begin();
  LOG(INFO) << "HX8357->Begin() OK";

  CHECK(xTaskCreate(&tft_test_task, "TFT Test", TaskStacks::kDefault, display,
                    1, nullptr));

  // InitLvgl(display);
  // LOG(INFO) << "InitLvgl() OK);
  // CHECK(xTaskCreate(&RunLvglDemo, "LVGL Demo", TaskStacks::kDefault,
  //                         nullptr, 1, nullptr));
  LOG(INFO) << "Startup complete.";
  vTaskDelete(nullptr);
}

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

  xTaskCreate(&StartupTask, "STARTUP", TaskStacks::kDefault, nullptr,
              TaskPriorities::kStartup, nullptr);
  vTaskStartScheduler();
  panic("scheduler died :(");
}

}  // namespace tplp

int main() { return tplp::main(); }
