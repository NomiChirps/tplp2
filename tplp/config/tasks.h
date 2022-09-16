#ifndef TPLP_CONFIG_TASKS_H_
#define TPLP_CONFIG_TASKS_H_

#include "FreeRTOSConfig.h"

namespace tplp {

struct TaskPriorities {
  // Just so we don't change it in one place and not update the other.
  static_assert(configMAX_PRIORITIES == 16);
  // FreeRTOS daemon should have very high priority.
  static_assert(configTIMER_TASK_PRIORITY == 15);

  // Logging is high priority. This means that *all* LOG() calls
  // from lower-priority tasks will block until the message is flushed.
  // Maybe later we can reduce this, but for debugging it's essential.
  static constexpr int kPicolog = 14;

  // Allow startup to finish without anything else getting in the way.
  // Except logging, of course.
  static constexpr int kStartup = 13;

  // Printer logic.
  static constexpr int kPaperController = 12;

  static constexpr int kRuntimeStats = 8;  // don't care

  // User-interface peripherals are low priority, but not the lowest.
  static constexpr int kHX8357 = 4;
  // I2cController needs work; it busy-waits a lot.
  static constexpr int kI2cController0 = 3;

  // Lightweight task responsible for keeping track of any background work
  // launched from the UI and calling UI callbacks.
  static constexpr int kUiWorker = 3;

  // Any remaining CPU time goes into making the UI more responsive.
  static constexpr int kLvglTimerHandler = 1;

  // FreeRTOS idle task runs at priority 0.
};

struct TaskStacks {
  static constexpr int kTESTONLY = 1024;
  static constexpr int kLvglTimerHandler = 2048;
  static constexpr int kHX8357 = 512;
  static constexpr int kI2cController = 1024;
  static constexpr int kRuntimeStats = 512;
  static constexpr int kStartup = 1024;
  static constexpr int kUiWorker = 1024;
  static constexpr int kPaperController = 1024;

  // don't reduce further unless you enjoy heisenbugs
  static constexpr int kPicolog = 1024;
};

}  // namespace tplp

#endif  // TPLP_CONFIG_TASKS_H_