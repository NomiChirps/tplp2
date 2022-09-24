#ifndef TPLP_UTIL_H_
#define TPLP_UTIL_H_

#include <cstdint>
#include <type_traits>

namespace tplp {

template <typename FnPtr,
          typename = typename std::enable_if<std::is_function<
              typename std::remove_pointer<FnPtr>::type>::value>>
bool IsInFlash(FnPtr ptr) {
  return (reinterpret_cast<intptr_t>(ptr) & 0xf0000000) == 0x10000000;
}

template <typename T, int N>
class RingBuffer {
 public:
  explicit RingBuffer(T initial_value) : begin_(0) {
    for (T& x : x_) {
      x = initial_value;
    }
  }

  T& operator[](int i) { return x_[(begin_ + i) % N]; }
  const T& operator[](int i) const { return x_[(begin_ + i) % N]; }

  // returns the back element that fell off
  T push_front(T x) {
    std::swap(x, x_[--begin_ % N]);
    return x;
  }

 private:
  T x_[N];
  uint32_t begin_;
};

}  // namespace tplp

#endif  // TPLP_UTIL_H_