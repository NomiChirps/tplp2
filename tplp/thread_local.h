#include "FreeRTOS.h"
#include "etl/flat_map.h"
#include "task.h"
#include "tplp/config.h"

namespace tplp {

// This is a nice C++ wrapper for the FreeRTOS thread-local storage
// implementation. It uses a single FreeRTOS thread-local storage pointer
// (`RtosTlsIndex`) from the array available to each task.
//
// Creating or deleting a `ThreadLocal` requires heap allocation and is
// relatively slow. Accessing the contents costs two indirections and a binary
// search on top of the FreeRTOS implementation.
template <typename T, int RtosTlsIndex = 0>
class ThreadLocal {
  static_assert(RtosTlsIndex < configNUM_THREAD_LOCAL_STORAGE_POINTERS,
                "RtosTlsIndex out of range");

 public:
  static constexpr int kMaxThreadLocals = 8;

 private:
  // TODO: can probably speed this up AND make it simpler by just using a sorted
  // vector of key-value pairs...
  using container_t = etl::flat_map<ThreadLocal<T>*, T*, kMaxThreadLocals>;

  // This should all be SMP-safe because the same task/thread will not be
  // scheduled on both cores simultaneously.
  static container_t* Fetch() {
    return static_cast<container_t*>(
        pvTaskGetThreadLocalStoragePointer(nullptr, RtosTlsIndex));
  }
  static container_t* InitAndFetch() {
    container_t* p = Fetch();
    if (!p) {
      p = new container_t;
      vTaskSetThreadLocalStoragePointer(nullptr, RtosTlsIndex, p);
    }
    return p;
  }

 public:
  explicit ThreadLocal(const T& initial_value) {
    container_t* p = InitAndFetch();
    p->insert({this, new T(initial_value)});
  }
  explicit ThreadLocal(T&& initial_value) {
    container_t* p = InitAndFetch();
    p->insert({this, new T(std::move(initial_value))});
  }
  ~ThreadLocal() {
    container_t* p = Fetch();
    auto it = p->find(this);
    delete it->second;
    p->erase(it);
  }

  T& get() { return *Fetch()->at(this); }

  const T& get() const { return *Fetch()->at(this); }
};

}  // namespace tplp