#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

void led_task(void*) {
    const TickType_t DELAY = 100 / portTICK_PERIOD_MS;
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
	vTaskDelay(DELAY);
        gpio_put(LED_PIN, 0);
	vTaskDelay(DELAY);
    }
}

void print_task(void*) {
    const TickType_t DELAY = 1000 / portTICK_PERIOD_MS;
    stdio_init_all();
    int n = 0;
    while(true) {
	printf("Hello, world! %d\n", n++);
	vTaskDelay(DELAY);
    }
}

int main() {
	// TODO: need any "hardware setup" for rtos?
	xTaskCreate(&led_task, "led", 1024, nullptr, 1, nullptr);
	xTaskCreate(&print_task, "print", 1024, nullptr, 1, nullptr);
	vTaskStartScheduler();
}
