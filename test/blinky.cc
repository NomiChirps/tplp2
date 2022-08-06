#include <cstdio>

#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/time.h"

int main() {
  stdio_init_all();
  const int kLed = 25;
  gpio_init(kLed);
  gpio_set_dir(kLed, GPIO_OUT);
  int value = 1;
  for (;;) {
    gpio_put(kLed, value);
    value = 1 - value;
    sleep_ms(500);
    printf("Meow!\n");
  }
}