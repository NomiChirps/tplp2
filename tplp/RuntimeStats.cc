#include "tplp/RuntimeStats.h"

#include <malloc.h>

#include <algorithm>
#include <cstdio>
#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl/lvgl.h"  // for heap monitoring
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"
#include "tplp/rtos_util.h"

namespace tplp {

class StatsTask {
 private:
  static constexpr int kMaxTasks = 32;
  static TaskStatus_t task_status[kMaxTasks];
  static uint32_t total_run_time;

 public:
  static void stats_task(void*) {
    LOG(INFO) << "stats_task started.";
    for (;;) {
      {
        struct mallinfo heap_stats = mallinfo();
        LOG(INFO) << "System heap arena=" << heap_stats.arena
                  << " uordblks=" << heap_stats.uordblks
                  << " fordblks=" << heap_stats.fordblks;
      }
      {
        HeapStats_t freertos_heap_stats;
        vPortGetHeapStats(&freertos_heap_stats);
        LOG(INFO) << "FreeRTOS heap size=" << configTOTAL_HEAP_SIZE
                  << " free=" << freertos_heap_stats.xAvailableHeapSpaceInBytes
                  << " low_water="
                  << freertos_heap_stats.xMinimumEverFreeBytesRemaining
                  << " largest="
                  << freertos_heap_stats.xSizeOfLargestFreeBlockInBytes
                  << " num_allocs="
                  << freertos_heap_stats.xNumberOfSuccessfulAllocations
                  << " num_frees="
                  << freertos_heap_stats.xNumberOfSuccessfulFrees;
      }
      if (lv_is_initialized()) {
        lv_mem_monitor_t lvgl_heap;
        lv_mem_monitor(&lvgl_heap);
        LOG(INFO) << "LVGL heap total_size=" << lvgl_heap.total_size
                  << " free_size=" << lvgl_heap.free_size
                  << " free_biggest_size=" << lvgl_heap.free_biggest_size
                  << " max_used=" << lvgl_heap.max_used;
      }
      {
        int column_width[] = {/*name=*/configMAX_TASK_NAME_LEN,
                              /*time total=*/7, /*time percent=*/4,
                              /*min free stack=*/6};
        int num_tasks =
            uxTaskGetSystemState(task_status, kMaxTasks, &total_run_time);
        int total_num_tasks = uxTaskGetNumberOfTasks();
        if (num_tasks != total_num_tasks) {
          LOG(WARNING) << "Only " << num_tasks << " tasks shown of "
                       << uxTaskGetNumberOfTasks() << " running.";
        }
        std::sort(task_status, task_status + num_tasks,
                  [](const TaskStatus_t& a, const TaskStatus_t& b) {
                    return a.xTaskNumber < b.xTaskNumber;
                  });
        total_run_time /= 100;
        LOG(INFO) << "FreeRTOS Task Stats: Task name; Total runtime; Percent "
                     "runtime; Stack high-water mark";
        for (int i = 0; i < num_tasks; ++i) {
          TaskStatus_t& task = task_status[i];
          LOG(INFO) << std::setw(column_width[0]) << task.pcTaskName
                    << std::right << std::setw(column_width[1])
                    << static_cast<int>(task.ulRunTimeCounter / 1000)
                    << std::setw(0) << "ms" << std::setw(column_width[2])
                    << static_cast<int>(task.ulRunTimeCounter / total_run_time)
                    << std::setw(0) << '%' << std::setw(column_width[3])
                    << static_cast<unsigned int>(task.usStackHighWaterMark)
                    << std::setw(0) << " bytes";
        }
      }
      vTaskDelay(MillisToTicks(30'000));
    }
  }

 private:
};

TaskStatus_t StatsTask::task_status[];
uint32_t StatsTask::total_run_time;

void StartRuntimeStatsReportingTask(int priority) {
  CHECK(xTaskCreate(&StatsTask::stats_task, "RuntimeStats", TaskStacks::kRuntimeStats,
                    nullptr, priority, nullptr));
}
}  // namespace tplp