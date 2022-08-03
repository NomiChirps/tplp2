#include "hardware/gpio.h"
#include "pico/time.h"

int main() {
  const int kLed = 25;
  gpio_init(kLed);
  gpio_set_dir(kLed, GPIO_OUT);
  int value = 1;
  for (;;) {
    gpio_put(kLed, value);
    value = 1 - value;
    sleep_ms(200);
  }
}