#include "tplp/RuntimeStats.h"

#include <malloc.h>

#include <algorithm>
#include <cstdio>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl/lvgl.h"  // for heap monitoring
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"

using std::chrono_literals::operator""ms;

namespace tplp {

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

void StartRuntimeStatsReportingTask(int priority) {
  tplp_assert(xTaskCreate(&StatsTask::stats_task, "print_stats",
                          TaskStacks::kDefault, nullptr, priority,
                          nullptr) == pdPASS);
}
}  // namespace tplp