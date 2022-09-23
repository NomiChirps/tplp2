#ifndef TPLP_CONFIG_TASKS_H_
#define TPLP_CONFIG_TASKS_H_

#include <cstdint>

#include "FreeRTOSConfig.h"

namespace tplp {

// Numerically-lower values indicate a higher priority. Hardware priorities
// range from 0 (highest priority) to 255 (lowest priority) though only the
// top 2 bits are significant on ARM Cortex-M0+.
struct IrqPriorities {
  // TODO: specify all of our irq priorities explicitly
  static constexpr uint8_t kHighest = 0x00;
  static constexpr uint8_t kHigher = 0x40;
  static constexpr uint8_t kDefault = 0x80;  // == PICO_DEFAULT_IRQ_PRIORITY
  static constexpr uint8_t kLowest = 0xc0;

  static constexpr uint8_t kPaperController = kHigher;
};

// Allocation of the 4 available hardware alarms on RP2040.
// Recall that for IRQs at the same priority, the lower numbered one
// takes precedence.
struct HardwareAlarms {
  static constexpr uint8_t kPaperController = 0;
};

// Allocation of the 8 available PWM slices.
struct PwmSlices {
  static constexpr int kMotorA = 0;
  static constexpr int kMotorB = 1;
};

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
  static constexpr int kPaperControllerLoop = 12;
  static constexpr int kPaperControllerCmd = 11;

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
  static constexpr int kLvglTimerHandler = 1024;
  static constexpr int kHX8357 = 512;
  static constexpr int kI2cController = 1024;
  static constexpr int kRuntimeStats = 512;
  static constexpr int kStartup = 1024;
  static constexpr int kUiWorker = 512;
  static constexpr int kPaperControllerCmd = 512;
  static constexpr int kPaperControllerLoop = 512;

  // don't reduce further unless you enjoy heisenbugs
  static constexpr int kPicolog = 1024;
};

}  // namespace tplp

#endif  // TPLP_CONFIG_TASKS_H_