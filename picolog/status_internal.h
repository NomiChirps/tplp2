#ifndef PICOLOG_STATUS_INTERNAL_H_
#define PICOLOG_STATUS_INTERNAL_H_

#include <string>

namespace util {

class [[nodiscard]] Status;
enum class StatusCode : int;

namespace status_internal {

// Reference-counted representation of Status data.
struct StatusRep {
  StatusRep(util::StatusCode code_arg, std::string_view message_arg)
      : ref(int32_t{1}), code(code_arg), message(message_arg) {}

  std::atomic<int32_t> ref;
  util::StatusCode code;
  std::string message;
};

util::StatusCode MapToLocalCode(int value);

// Returns a pointer to a newly-allocated string with the given `prefix`,
// suitable for output as an error message in assertion/`CHECK()` failures.
//
// This is an internal implementation detail for Abseil logging.
std::string* MakeCheckFailString(const util::Status* status,
                                 const char* prefix);

}  // namespace status_internal
}  // namespace util

#endif  // PICOLOG_STATUS_INTERNAL_H_