#ifndef TPLP_RTOS_UTILS_H_
#define TPLP_RTOS_UTILS_H_

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
}  // namespace tplp

#endif  // TPLP_RTOS_UTILS_H_