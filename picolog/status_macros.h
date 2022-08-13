#ifndef PICOLOG_STATUS_MACROS_H_
#define PICOLOG_STATUS_MACROS_H_

#include "picolog/status.h"

// TODO: use PICOLOG_PREDICT_FALSE, or whatever idk
#define STATUS_MACROS_PREDICT_FALSE(x) (__builtin_expect(x, 0))

namespace util {
namespace status_macros_internal {
using ::util::Status;
}
// Run a command that returns a util::Status.  If the called code returns an
// error status, return that status up out of this method too.
//
// Example:
//   RETURN_IF_ERROR(DoThings(4));
#define RETURN_IF_ERROR(expr)                                                \
  do {                                                                       \
    /* Using _status below to avoid capture problems if expr is "status". */ \
    ::util::status_macros_internal::Status _status = (expr);                 \
    if (STATUS_MACROS_PREDICT_FALSE(!_status.ok())) return _status;          \
  } while (0)

// TODO: reinstate this when StatusOr is implemented
// clang-format off
/*
// Internal helper for concatenating macro values.
#define STATUS_MACROS_CONCAT_NAME_INNER(x, y) x##y
#define STATUS_MACROS_CONCAT_NAME(x, y) STATUS_MACROS_CONCAT_NAME_INNER(x, y)

template <typename T>
Status DoAssignOrReturn(T& lhs, StatusOr<T> result) {
  if (result.ok()) {
    lhs = result.value();
  }
  return result.status();
}

#define ASSIGN_OR_RETURN_IMPL(status, lhs, rexpr) \
  Status status = DoAssignOrReturn(lhs, (rexpr)); \
  if (STATUS_MACROS__PREDICT_FALSE(!status.ok())) return status;

// Executes an expression that returns a util::StatusOr, extracting its value
// into the variable defined by lhs (or returning on error).
//
// Example: Assigning to an existing value
//   ValueType value;
//   ASSIGN_OR_RETURN(value, MaybeGetValue(arg));
//
// WARNING: ASSIGN_OR_RETURN expands into multiple statements; it cannot be used
//  in a single statement (e.g. as the body of an if statement without {})!
#define ASSIGN_OR_RETURN(lhs, rexpr) \
  ASSIGN_OR_RETURN_IMPL(             \
      STATUS_MACROS_CONCAT_NAME(_status_or_value, __COUNTER__), lhs, rexpr);
*/
// clang-format on
}  // namespace util

#endif  // PICOLOG_STATUS_MACROS_H_