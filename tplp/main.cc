#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/stdlib.h"
#include "tplp/RuntimeStats.h"
#include "tplp/SharpLCD/SharpLCD.h"
#include "tplp/SpiManager.h"
#include "tplp/graphics/graphics.h"
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"
#include "tplp/types.h"
#include "tplp/ws2812.h"

using std::chrono_literals::operator""ms;

namespace tplp {

void led_task(void*) {
  DebugLog("led_task started.");
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

// TODO: move this to its own file.
void StartupTask(void*) {
  DebugLog("Begin startup...");
  // Other tasks won't run during startup because this one is using the reserved
  // highest priority.

  SpiManager* spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/std::nullopt);
  DebugLog("SpiManager::Init() OK");

  SharpLCD* display = new SharpLCD(spi0_manager);
  display->Begin(Pins::LCD_CS, TaskPriorities::kSharpLcdToggleVcom);
  DebugLog("SharpLCD->Begin() OK");
  InitLvgl(display);
  DebugLog("InitLvgl() OK");
  tplp_assert(xTaskCreate(&led_task, "blinky", TaskStacks::kDefault, nullptr, 1,
                          nullptr) == pdPASS);
  StartRuntimeStatsReportingTask(/*priority=*/1);
  tplp_assert(xTaskCreate(&RunLvglDemo, "LVGL Demo", TaskStacks::kDefault,
                          nullptr, 1, nullptr) == pdPASS);
  DebugLog("Startup complete.");
  vTaskDelete(nullptr);
}

int main() {
  stdio_init_all();
  // TODO: add a build timestamp and version line at bootup
  // printf("tplp2 built %s at %s", TPLP_BUILD_TIMESTAMP, TPLP_VERSION_ID);

  xTaskCreate(&StartupTask, "STARTUP", TaskStacks::kDefault, nullptr,
              configMAX_PRIORITIES - 1, nullptr);
  vTaskStartScheduler();
  panic("scheduler died :(");
}

}  // namespace tplp

int main() { return tplp::main(); }
