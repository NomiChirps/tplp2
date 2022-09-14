#ifndef ABSL_STRINGS_STRING_VIEW_H_
#define ABSL_STRINGS_STRING_VIEW_H_

#include <string_view>  // IWYU pragma: export

#include "absl/base/macros.h"

namespace absl {
ABSL_NAMESPACE_BEGIN
using string_view = std::string_view;
ABSL_NAMESPACE_END
}  // namespace absl

namespace absl {
ABSL_NAMESPACE_BEGIN

// ClippedSubstr()
//
// Like `s.substr(pos, n)`, but clips `pos` to an upper bound of `s.size()`.
// Provided because std::string_view::substr throws if `pos > size()`
inline string_view ClippedSubstr(string_view s, size_t pos,
                                 size_t n = string_view::npos) {
  pos = (std::min)(pos, static_cast<size_t>(s.size()));
  return s.substr(pos, n);
}

// NullSafeStringView()
//
// Creates an `absl::string_view` from a pointer `p` even if it's null-valued.
// This function should be used where an `absl::string_view` can be created from
// a possibly-null pointer.
constexpr string_view NullSafeStringView(const char* p) {
  return p ? string_view(p) : string_view();
}

ABSL_NAMESPACE_END
}  // namespace absl

#endif  // ABSL_STRINGS_STRING_VIEW_H_