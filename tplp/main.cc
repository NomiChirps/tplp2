#include <chrono>
#include <cstdio>

#include "Adafruit_SharpMem.h"
#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "task.h"
#include "tplp/util.h"

using std::chrono_literals::operator""ms;

namespace tplp {

struct Pins {
  static const int LCD_CS = 6;
  static const int SPI_SCLK = 18;
  static const int SPI_MOSI = 19;
};

// TODO: make sure this doesn't use any hard delays (scheduler)
// TODO: see if this is using hw spi support; if not, make it.
Adafruit_SharpMem display(Pins::SPI_SCLK, Pins::SPI_MOSI, Pins::LCD_CS, 144,
                          168);

void lcd_task(void *) {
  display.begin();
  display.clearDisplay();

  while (true) {
    display.fillScreen(0);
    display.refresh();
    vTaskDelay(as_ticks(1000ms));
    display.fillScreen(1);
    display.refresh();
    vTaskDelay(as_ticks(1000ms));
  }
}

void led_task(void *) {
  const int LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  while (true) {
    gpio_put(LED_PIN, 1);
    vTaskDelay(as_ticks(100ms));
    gpio_put(LED_PIN, 0);
    vTaskDelay(as_ticks(100ms));
  }
}

int main() {
  xTaskCreate(&tplp::lcd_task, "lcd", 1024, nullptr, 1, nullptr);
  xTaskCreate(&tplp::led_task, "led", 1024, nullptr, 1, nullptr);
  vTaskStartScheduler();
  return 0;
}

}  // namespace tplp

int main() { return tplp::main(); }
