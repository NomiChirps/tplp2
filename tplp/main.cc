#include <chrono>
#include <cstdio>

// TODO: figure out how to add FreeRTOS/ prefix to these headers
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "lib/SharpLCD/SharpLCD.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"
#include "tplp/util.h"
#include "tplp/ws2812.h"


using std::chrono_literals::operator""ms;

extern "C" {

// FreeRTOS assertion failure handler
void vAssertCalled(const char *const file, unsigned long line) {
  printf("Assertion failed at: %s:%lu\n", file, line);
  // Leave interrupts enabled so we can still get back to the bootloader via
  // the USB serial magic thing.
  for (;;) {
  }
}

void FreeRTOS_ConfigureTimeForRunTimeStats() {}

unsigned long FreeRTOS_GetRunTimeCounterValue() {
  return to_us_since_boot(get_absolute_time());
}
}

namespace tplp {

struct Pins {
  static const uint LCD_CS = 8;
  static const uint SPI_SCLK = 18;
  static const uint SPI_MOSI = 19;
  static const uint NEOPIXEL = 16;
};

void lcd_task(void *) {
  SharpLCD display(spi0, Pins::SPI_SCLK, Pins::SPI_MOSI, Pins::LCD_CS);
  display.Begin();
  display.Clear();
  SharpLCD::FrameBuffer white = display.AllocateNewFrameBuffer();
  SharpLCD::FrameBuffer black = display.AllocateNewFrameBuffer();
  white.Clear(0);
  black.Clear(1);

  while (true) {
    // printf("Drawing white\n");
    display.DrawFrameBufferBlocking(white);
    // vTaskDelay(as_ticks(2000ms));
    // printf("Drawing black\n");
    display.DrawFrameBufferBlocking(black);
    // vTaskDelay(as_ticks(2000ms));
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

  xTaskCreate(&tplp::lcd_task, "lcd", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::led_task, "led", 1024, nullptr, 2, nullptr);
  xTaskCreate(&tplp::stats_task, "stats", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();
  return 0;
}

}  // namespace tplp

int main() { return tplp::main(); }
