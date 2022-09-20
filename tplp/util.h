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

}  // namespace tplp

#endif  // TPLP_UTIL_H_