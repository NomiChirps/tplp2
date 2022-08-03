#ifndef TPLP_THREAD_LOCAL_H_
#define TPLP_THREAD_LOCAL_H_

#include <array>
#include <functional>
#include <memory>
#include <optional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "picolog/picolog.h"

namespace tplp {

// Provides simple thread-local storage under FreeRTOS, without using any of the
// FreeRTOS-provided TLS pointers. Methods of this class are NOT thread-safe.
//
// `MaxSharing` is the maximum number of tasks that can store their separate
// values in one instance of ThreadLocal.
//
// ThreadLocal acts as a value object, so copying it also copies the contained
// value for each task (if any).
//
// Accessing ThreadLocal outside of a task context is allowed; in this case
// all such access shares one value of `T`. This does not count towards
// `MaxSharing`.
//
// FIXME: unclear if this rewrite actually works. test it or something.
template <typename T, int MaxSharing = 4>
class ThreadLocal {
 private:
  using container_t =
      std::array<std::pair<TaskHandle_t, std::optional<T>>, MaxSharing>;

  container_t p_;
  int count_;
  std::optional<T> outside_task_value_;

 public:
  using initializer_t = std::function<T()>;
  const initializer_t initializer_;

 public:
  explicit ThreadLocal(const initializer_t& initializer)
      : p_{},  // value-initialize
        count_(0),
        outside_task_value_(std::nullopt),
        initializer_(initializer) {}

  // Default operations are fine. Thanks std::optional!
  ThreadLocal(const ThreadLocal&) = default;
  ThreadLocal& operator=(const ThreadLocal& other) = default;
  ThreadLocal(ThreadLocal&&) = default;
  ThreadLocal& operator=(ThreadLocal&&) = default;
  ~ThreadLocal() = default;

  // Returns a reference to a thread-local value of `T` owned by this object,
  // initializing it first if necessary. Not thread-safe, i.e., do not call
  // `get_or_init` on the same instance of `ThreadLocal` from multiple tasks
  // simultaneously.
  T& get_or_init() {
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (!task) {
      if (outside_task_value_) return *outside_task_value_;
      outside_task_value_ = initializer_();
      return *outside_task_value_;
    }
    // Linear search will be faster than any map lookup if `MaxSharing` is
    // small, as it usually should be.
    for (int i = 0; i < count_; ++i) {
      if (p_[i].first == task) {
        return *p_[i].second;
      }
    }
    CHECK_LT(count_, MaxSharing) << "ThreadLocal size exceeded MaxSharing";
    p_[count_].first = task;
    p_[count_].second = initializer_();
    return *p_[count_++].second;
  }
};

}  // namespace tplp

#endif  // TPLP_THREAD_LOCAL_H_