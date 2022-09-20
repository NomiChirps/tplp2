#ifndef TPLP_NUMBERS_H_
#define TPLP_NUMBERS_H_

#include <cstdint>

// struct rational32_t {
//   explicit rational32_t() : num(0), den(1) {}
//   explicit rational32_t(int32_t num) : num(num), den(1) {}
//   rational32_t(int32_t num, int32_t den) : num(num), den(den) {}

//   int32_t num;
//   int32_t den;

//   bool operator<(const rational32_t& rhs) {
//     return ((int64_t)num * rhs.den) < ((int64_t)den * rhs.num);
//   }

//   bool operator>(const rational32_t& rhs) {
//     return ((int64_t)num * rhs.den) > ((int64_t)den * rhs.num);
//   }

//   int32_t operator*(int32_t rhs) const { return ((int64_t)num * rhs) / den; }
// };

namespace util {
template <typename T>
int signum(T val) {
  return (T(0) < val) - (val < T(0));
}
}  // namespace util

#endif  // TPLP_NUMBERS_H_