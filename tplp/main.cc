#include <chrono>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/stdlib.h"
#include "tplp/SharpLCD/SharpLCD.h"
#include "tplp/SpiManager.h"
#include "tplp/assert.h"
#include "tplp/graphics/graphics.h"
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"
#include "tplp/types.h"
#include "tplp/ws2812.h"

using std::chrono_literals::operator""ms;

namespace tplp {

void led_task(void *) {
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

void neopixel_task(void *) {
  // TODO: play with neopixel
  // ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq,
  // bool rgbw)
}

void stats_task(void *) {
  DebugLog("stats_task started.");
  static char buf[1024];
  for (;;) {
    vTaskDelay(as_ticks(10'000ms));
    vTaskGetRunTimeStats(buf);
    // hm. indentation would be nice.
    DebugLog("\n-- FreeRTOS Stats --\n%s--------------------", buf);
    // TODO: uxTaskGetSystemState and print stack size info
    // TODO: vPortGetHeapStats and print freertos heap info
    // TODO: malloc/free to try and get pico heap info
  }
}

int main() {
  stdio_init_all();
  // TODO: add a build timestamp and version line at bootup
  // printf("tplp2 built %s at %s", TPLP_BUILD_TIMESTAMP, TPLP_VERSION_ID);
  DebugLogStaticInit();

  DebugLog("Hello.");
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
  SpiManager *spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/0);
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
  DebugLog("SpiManager::Init() OK");

  SharpLCD *display = new SharpLCD(spi0_manager);
  display->Begin(Pins::LCD_CS, TaskPriorities::kSharpLcdToggleVcom);
  DebugLog("SharpLCD->Begin() OK");
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
  InitLvgl(display);
  DebugLog("InitLvgl() OK");
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);

  tplp_assert(xTaskCreate(&tplp::led_task, "blinky",
                          TplpConfig::kDefaultTaskStackSize, nullptr, 1,
                          nullptr) == pdPASS);
  tplp_assert(xTaskCreate(&tplp::stats_task, "print_stats",
                          TplpConfig::kDefaultTaskStackSize, nullptr, 1,
                          nullptr) == pdPASS);
  tplp_assert(xTaskCreate(&tplp::RunLvglDemo, "LVGL Demo",
                          TplpConfig::kDefaultTaskStackSize, nullptr, 1,
                          nullptr) == pdPASS);
  DebugLog("Setup complete. Starting scheduler.");
  // does not return unless we kill the scheduler, or it fails to start
  vTaskStartScheduler();
  panic("scheduler died");
}

}  // namespace tplp

int main() { return tplp::main(); }
