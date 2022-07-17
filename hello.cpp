#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Adafruit_SharpMem.h"

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

// TODO: don't need to be #defines
// any pins can be used
#define SHARP_SCK  10
#define SHARP_MOSI 11
#define SHARP_SS   13

// Set the size of the display here, e.g. 144x168!
// TODO: make sure this doesn't use any hard delays (scheduler)
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 144, 168);
// The currently-available SHARP Memory Display (144x168 pixels)
// requires > 4K of microcontroller RAM; it WILL NOT WORK on Arduino Uno
// or other <4K "classic" devices!  The original display (96x96 pixels)
// does work there, but is no longer produced.

class LcdTest {
public:
	const int BLACK = 0;
	const int WHITE = 1;
	const int minorHalfSize = min(display.width(), display.height()) / 2; 
	const int DELAY = 100;
void testdrawline() {
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, BLACK);
    display.refresh();
  }
  vTaskDelay(DELAY);

  display.clearDisplay();
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, BLACK);
    display.refresh();
  }
  vTaskDelay(DELAY);

  display.clearDisplay();
  for (int i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, BLACK);
    display.refresh();
  }
  vTaskDelay(DELAY);

  display.clearDisplay();
  for (int i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  vTaskDelay(DELAY);
}

void testdrawrect(void) {
  for (int i=0; i<minorHalfSize; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, BLACK);
    display.refresh();
  }
}

void testfillrect(void) {
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize; i+=3) {
    // alternate colors
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawroundrect(void) {
  for (int i=0; i<minorHalfSize/2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, BLACK);
    display.refresh();
  }
}

void testfillroundrect(void) {
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize/2; i+=2) {
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawtriangle(void) {
  for (int i=0; i<minorHalfSize; i+=5) {
    display.drawTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, BLACK);
    display.refresh();
  }
}

void testfilltriangle(void) {
  uint8_t color = BLACK;
  for (int i=minorHalfSize; i>0; i-=5) {
    display.fillTriangle(display.width()/2  , display.height()/2-i,
                         display.width()/2-i, display.height()/2+i,
                         display.width()/2+i, display.height()/2+i, color & 1);
    display.refresh();
    color++;
  }
}

void testdrawchar(void) {
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.cp437(true);

  for (int i=0; i < 256; i++) {
    if (i == '\n') continue;
    display.write(i);
  }
  display.refresh();
}
};

void lcd_task(void*) {
	// chip select
	const uint DISPLAY_EN = 9;
	gpio_init(DISPLAY_EN);
	gpio_set_dir(DISPLAY_EN, GPIO_OUT);
	gpio_put(DISPLAY_EN, 1);

	LcdTest test;
	  // start & clear the display
  display.begin();
  display.clearDisplay();
  const int DELAY = 100;

  while (true) {
  // draw a single pixel
  display.drawPixel(10, 10, test.BLACK);
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  // draw many lines
  test.testdrawline();
  vTaskDelay(DELAY);
  display.clearDisplay();

  // draw rectangles
  test.testdrawrect();
  vTaskDelay(DELAY);
  display.clearDisplay();

  // draw multiple rectangles
  test.testfillrect();
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  // draw a circle, 10 pixel radius
  display.fillCircle(display.width()/2, display.height()/2, 10, test.BLACK);
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  test.testdrawroundrect();
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  test.testfillroundrect();
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  test.testdrawtriangle();
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  test.testfilltriangle();
  display.refresh();
  vTaskDelay(DELAY);
  display.clearDisplay();

  test.testdrawchar();
  display.refresh();
  }
}

int main() {
	// TODO: need any "hardware setup" for rtos?
	xTaskCreate(&led_task, "led", 1024, nullptr, 1, nullptr);
	xTaskCreate(&print_task, "print", 1024, nullptr, 1, nullptr);
	xTaskCreate(&lcd_task, "lcd", 1024, nullptr, 1, nullptr);
	vTaskStartScheduler();
}
