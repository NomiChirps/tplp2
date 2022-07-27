#include <chrono>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "pico/stdlib.h"
#include "tplp/SharpLCD/SharpLCD.h"
#include "tplp/SpiManager.h"
#include "tplp/graphics/graphics.h"
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"
#include "tplp/types.h"
#include "tplp/ws2812.h"

using std::chrono_literals::operator""ms;

extern "C" {
// FreeRTOS assertion failure handler
void FreeRTOS_AssertionFailed(const char *const file, unsigned long line) {
  panic("Assertion failed at: %s:%lu\n", file, line);
}

void FreeRTOS_ConfigureTimeForRunTimeStats() {
  // nothing to do; pico bootloader already starts the relevant timers.
}

unsigned long FreeRTOS_GetRunTimeCounterValue() {
  return to_us_since_boot(get_absolute_time());
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
  panic("Stack overflow in task: %s\n", name);
}

void lvgl_assertion_failed() { panic("lvgl assertion failed"); }

}  // extern "C"

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
    printf("-- FreeRTOS Stats --\n");
    vTaskGetRunTimeStats(buf);
    puts(buf);
    printf("--------------------\n");
    stdio_flush();
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
  printf("Hello!\n");

  SpiManager *spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/0);

  SharpLCD *display = new SharpLCD(spi0_manager);
  display->Begin(Pins::LCD_CS);
  DebugLog("SharpLCD->Begin OK");
  InitLvgl(display);
  DebugLog("InitLvgl OK");

  xTaskCreate(&tplp::led_task, "led", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::stats_task, "stats", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::RunLvglDemo, "LVGL Demo", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::ToggleVcomTask, "ToggleVCOM", 1024, display,
              TaskPriorities::kSharpLcdToggleVcom, nullptr);
  DebugLog("Setup complete. Starting scheduler.");
  vTaskStartScheduler();
  // does not return unless we kill the scheduler
  for (;;)
    ;
}

}  // namespace tplp

int main() { return tplp::main(); }
