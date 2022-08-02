#ifndef TPLP_THREAD_LOCAL_H_
#define TPLP_THREAD_LOCAL_H_

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "etl/flat_map.h"
#include "tplp/logging.h"
#include "tplp/tplp_config.h"

namespace tplp {

// This is a nice C++ wrapper for the FreeRTOS thread-local storage
// implementation. It uses a single FreeRTOS thread-local storage pointer
// (`RtosTlsIndex`) from the array available to each task.
template <typename T, int RtosTlsIndex = 0>
class ThreadLocal {
  static_assert(RtosTlsIndex < configNUM_THREAD_LOCAL_STORAGE_POINTERS,
                "RtosTlsIndex out of range");

 public:
  static constexpr int kMaxThreadLocals = 8;

 private:
  // TODO: can probably speed this up AND make it simpler by just using a sorted
  // vector of key-value pairs...
  // TODO: should make these statically allocated, at least (kMaxTasks?)
  using container_t = etl::flat_map<ThreadLocal<T>*, T*, kMaxThreadLocals>;

  // This should all be SMP-safe because the same task/thread will not be
  // scheduled on both cores simultaneously.
  static container_t* InitAndFetch() {
    container_t* p = static_cast<container_t*>(
        pvTaskGetThreadLocalStoragePointer(nullptr, RtosTlsIndex));
    if (!p) {
      p = new container_t;
      vTaskSetThreadLocalStoragePointer(nullptr, RtosTlsIndex, p);
      LOG(INFO) << "ThreadLocal storage initialized.";
    }
    return p;
  }

 public:
  using initializer_t = std::function<T()>;

  explicit ThreadLocal(const initializer_t& initializer)
      : initializer_(initializer) {}

  ~ThreadLocal() {
    if (!xTaskGetCurrentTaskHandle()) {
      LOG(FATAL) << "ThreadLocal::~ThreadLocal() called outside a task context";
    }
    container_t* p = InitAndFetch();
    auto it = p->find(this);
    delete it->second;
    p->erase(it);
  }

  ThreadLocal(const ThreadLocal&) = delete;
  ThreadLocal& operator=(const ThreadLocal&) = delete;

  T& get_or_init() {
    if (!xTaskGetCurrentTaskHandle()) {
      LOG(FATAL) << "ThreadLocal::get_or_init() called outside a task context";
    }
    container_t* p = InitAndFetch();
    auto it = p->find(this);
    if (it == p->end()) {
      return *p->insert({this, new T(initializer_())}).first->second;
    }
    return *it->second;
  }

 private:
  const std::function<T()> initializer_;
};

}  // namespace tplp

#endif  // TPLP_THREAD_LOCAL_H_