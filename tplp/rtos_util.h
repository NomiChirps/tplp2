#ifndef TPLP_RTOS_UTIL_H_
#define TPLP_RTOS_UTIL_H_

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "picolog/picolog.h"

namespace tplp {

// Scoped deleter for SemaphoreHandle_t.
// Example usage: ScopedSemaphoreHandle sem(xSemaphoreCreateBinary());
class ScopedSemaphoreHandle {
 public:
  // CHECK fails if null.
  explicit ScopedSemaphoreHandle(SemaphoreHandle_t sem)
      : sem_(CHECK_NOTNULL(sem)) {}
  ~ScopedSemaphoreHandle() { vSemaphoreDelete(sem_); }
  ScopedSemaphoreHandle(const ScopedSemaphoreHandle&) = delete;
  ScopedSemaphoreHandle& operator=(const ScopedSemaphoreHandle&) = delete;

  operator SemaphoreHandle_t() const { return sem_; }

 private:
  const SemaphoreHandle_t sem_;
};

// Releases a NON-RECURSIVE semaphore or mutex at end of scope.
// Example usage:
//
//     CHECK(xSemaphoreTake(mutex));
//     ScopedSemaphoreReleaser releaser(mutex);
//     ... do stuff ...
//     // xSemaphoreGive() is automatically called
class ScopedSemaphoreReleaser {
 public:
  explicit ScopedSemaphoreReleaser(SemaphoreHandle_t sem)
      : sem_(CHECK_NOTNULL(sem)) {}
  ~ScopedSemaphoreReleaser() { CHECK(xSemaphoreGive(sem_)); }
  ScopedSemaphoreReleaser(const ScopedSemaphoreReleaser&) = delete;
  ScopedSemaphoreReleaser& operator=(const ScopedSemaphoreReleaser&) = delete;

  // TODO:  need statusor so we can:
  // static util::StatusOr<ScopedSemaphoreReleaser> Take(sem, timeout);
 private:
  const SemaphoreHandle_t sem_;
};

// Waits until at least the given number of milliseconds have passed since boot.
// Scheduler must be running.
void EnsureTimeSinceBootMillis(int wait_until);

// Integer division, rounding up.
template <typename Int>
inline Int intdiv_ceil(Int x, Int y) {
  return (x / y) + (x % y != 0);
}

// Always rounds up, under the assumption that this is being used to calculate a
// delay time.
inline TickType_t MillisToTicks(TickType_t ms) {
  return intdiv_ceil<TickType_t>(ms * configTICK_RATE_HZ, 1000);
}

}  // namespace tplp

#endif  // TPLP_RTOS_UTIL_H_