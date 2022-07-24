#include <chrono>
#include <cstdio>
#include <memory>

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"
#include "tplp/SharpLCD/SharpLCD.h"
#include "tplp/SpiManager.h"
#include "tplp/config.h"
#include "tplp/types.h"
#include "tplp/util.h"
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

}  // extern "C"

namespace tplp {

SpiManager *global_spi0_manager;

void lcd_task(void *) {
  SharpLCD display(global_spi0_manager);
  display.Begin(Pins::LCD_CS);
  display.Clear();
  SharpLCD::FrameBuffer white = display.AllocateNewFrameBuffer();
  SharpLCD::FrameBuffer black = display.AllocateNewFrameBuffer();
  white.Clear(0);
  black.Clear(1);

  while (true) {
    display.DrawFrameBufferBlocking(white);
    display.DrawFrameBufferBlocking(black);
  }
}

void led_task(void *) {
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
  // ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq,
  // bool rgbw)
}

void button_task(void *) {
  // xQueueReceive(button_queue, void *const pvBuffer, TickType_t xTicksToWait);
}

void stats_task(void *) {
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

int main() {
  stdio_init_all();
  printf("Hello!\n");

  global_spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/0);

  xTaskCreate(&tplp::lcd_task, "lcd", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::led_task, "led", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::stats_task, "stats", 1024, nullptr, 1, nullptr);
  vTaskStartScheduler();
  return 0;
}

}  // namespace tplp

int main() { return tplp::main(); }
