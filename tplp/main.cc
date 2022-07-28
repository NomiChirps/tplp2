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
    DebugLog("\n-- FreeRTOS Stats --\n{}--------------------", buf);
  }
}

// TODO: move back to SharpLCD.cc ?
void ToggleVcomTask(void *param) {
  DebugLog("ToggleVcomTask started.");
  for (;;) {
    static_cast<SharpLCD *>(param)->ToggleVCOM();
    vTaskDelay(as_ticks(std::chrono::minutes(60)));
  }
}

int main() {
  stdio_init_all();
  DebugLog("Hello!");

  SpiManager *spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/0);
  DebugLog("SpiManager::Init() OK");

  SharpLCD *display = new SharpLCD(spi0_manager);
  display->Begin(Pins::LCD_CS);
  DebugLog("SharpLCD->Begin() OK");
  InitLvgl(display);
  DebugLog("InitLvgl() OK");

  xTaskCreate(&tplp::led_task, "blinky", TplpConfig::kDefaultTaskStackSize,
              nullptr, 1, nullptr);
  xTaskCreate(&tplp::stats_task, "print_stats",
              TplpConfig::kDefaultTaskStackSize, nullptr, 1, nullptr);
  xTaskCreate(&tplp::RunLvglDemo, "LVGL Demo",
              TplpConfig::kDefaultTaskStackSize, nullptr, 1, nullptr);
  xTaskCreate(&tplp::ToggleVcomTask, "ToggleVCOM",
              TplpConfig::kDefaultTaskStackSize, display,
              TaskPriorities::kSharpLcdToggleVcom, nullptr);
  DebugLog("Setup complete. Starting scheduler.");
  // does not return unless we kill the scheduler, or it fails to start
  vTaskStartScheduler();
  panic("scheduler died");
}

}  // namespace tplp

int main() { return tplp::main(); }
