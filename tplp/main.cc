#include <malloc.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl/lvgl.h"  // for heap monitoring
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

void led_task(void*) {
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

void neopixel_task(void*) {
  // TODO: play with neopixel
  // ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq,
  // bool rgbw)
}

class StatsTask {
 private:
  static constexpr int kMaxTasks = 32;
  static TaskStatus_t task_status[kMaxTasks];
  static uint32_t total_run_time;
  static constexpr int kBufSize = 1024;
  static char buf[kBufSize];
  static char* buf_head;
  static constexpr char* buf_tail = buf + kBufSize;

 public:
  static void stats_task(void*) {
    DebugLog("stats_task started.");
    for (;;) {
      {
        struct mallinfo heap_stats = mallinfo();
        DebugLog("System heap arena=%u uordblks=%u fordblks=%u",
                 heap_stats.arena, heap_stats.uordblks, heap_stats.fordblks);
      }
      {
        HeapStats_t freertos_heap_stats;
        vPortGetHeapStats(&freertos_heap_stats);
        DebugLog(
            "FreeRTOS heap size=%u free=%u low_water=%u largest=%u "
            "num_allocs=%u "
            "num_frees=%u",
            configTOTAL_HEAP_SIZE,
            freertos_heap_stats.xAvailableHeapSpaceInBytes,
            freertos_heap_stats.xMinimumEverFreeBytesRemaining,
            freertos_heap_stats.xSizeOfLargestFreeBlockInBytes,
            freertos_heap_stats.xNumberOfSuccessfulAllocations,
            freertos_heap_stats.xNumberOfSuccessfulFrees);
      }
      if (lv_is_initialized()) {
        lv_mem_monitor_t lvgl_heap;
        lv_mem_monitor(&lvgl_heap);
        DebugLog(
            "LVGL heap total_size=%u free_size=%u free_biggest_size=%u "
            "max_used=%u",
            lvgl_heap.total_size, lvgl_heap.free_size,
            lvgl_heap.free_biggest_size, lvgl_heap.max_used);
      }
      {
        buf_head = buf;
        int column_width[] = {/*name=*/configMAX_TASK_NAME_LEN,
                              /*time total=*/7, /*time percent=*/4,
                              /*min free stack=*/6};
        int num_tasks =
            uxTaskGetSystemState(task_status, kMaxTasks, &total_run_time);
        int total_num_tasks = uxTaskGetNumberOfTasks();
        if (num_tasks != total_num_tasks) {
          DebugLog("Warning: %d tasks shown of %u running.\n", num_tasks,
                   uxTaskGetNumberOfTasks());
        }
        std::sort(task_status, task_status + num_tasks,
                  [](const TaskStatus_t& a, const TaskStatus_t& b) {
                    return a.xTaskNumber < b.xTaskNumber;
                  });
        total_run_time /= 100;
        PrintToBuf("FreeRTOS Task Stats:\n");
        for (int i = 0; i < num_tasks; ++i) {
          TaskStatus_t& task = task_status[i];
          PrintToBuf("%-*s%*dms%*d%%%*u bytes\n", column_width[0],
                     task.pcTaskName, column_width[1],
                     static_cast<int>(task.ulRunTimeCounter / 1000),
                     column_width[2],
                     static_cast<int>(task.ulRunTimeCounter / total_run_time),
                     column_width[3],
                     static_cast<unsigned int>(task.usStackHighWaterMark));
        }
        DebugLog("%s", buf);
      }
      vTaskDelay(as_ticks(30'000ms));
    }
  }

 private:
  [[gnu::format(printf, 1, 2)]] static void PrintToBuf(const char* format,
                                                       ...) {
    va_list arglist;
    va_start(arglist, format);
    buf_head += vsnprintf(buf_head, buf_tail - buf_head, format, arglist);
    va_end(arglist);
  }
};

TaskStatus_t StatsTask::task_status[];
char StatsTask::buf[];
uint32_t StatsTask::total_run_time;
char* StatsTask::buf_head;

void StartupTask(void*) {
  DebugLog("Begin startup...");
  // Other tasks won't run during startup because this one is using the reserved
  // highest priority.

  SpiManager* spi0_manager =
      SpiManager::Init(TaskPriorities::kSpiManager0, spi0, 2'000'000,
                       Pins::SPI_SCLK, Pins::SPI_MOSI, /*miso=*/0);
  DebugLog("SpiManager::Init() OK");

  SharpLCD* display = new SharpLCD(spi0_manager);
  display->Begin(Pins::LCD_CS, TaskPriorities::kSharpLcdToggleVcom);
  DebugLog("SharpLCD->Begin() OK");
  InitLvgl(display);
  DebugLog("InitLvgl() OK");
  tplp_assert(xTaskCreate(&led_task, "blinky", TaskStacks::kDefault, nullptr, 1,
                          nullptr) == pdPASS);
  tplp_assert(xTaskCreate(&StatsTask::stats_task, "print_stats",
                          TaskStacks::kDefault, nullptr, 1, nullptr) == pdPASS);
  tplp_assert(xTaskCreate(&RunLvglDemo, "LVGL Demo", TaskStacks::kDefault,
                          nullptr, 1, nullptr) == pdPASS);
  DebugLog("Startup complete. Resuming all tasks.");
  vTaskDelete(nullptr);
}

int main() {
  stdio_init_all();
  // TODO: add a build timestamp and version line at bootup
  // printf("tplp2 built %s at %s", TPLP_BUILD_TIMESTAMP, TPLP_VERSION_ID);

  xTaskCreate(&StartupTask, "STARTUP", TaskStacks::kDefault, nullptr,
              configMAX_PRIORITIES - 1, nullptr);
  vTaskStartScheduler();
  panic("scheduler died :(");
}

}  // namespace tplp

int main() { return tplp::main(); }
