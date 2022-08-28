#include "picolog/statusor.h"

#include <cstdlib>
#include <utility>

#include "picolog/picolog.h"

namespace util {

BadStatusOrAccess::BadStatusOrAccess(util::Status status)
    : status_(std::move(status)) {}

BadStatusOrAccess& BadStatusOrAccess::operator=(BadStatusOrAccess&& other) {
  // Ensure assignment is correct regardless of whether this->InitWhat() has
  // already been called.
  other.InitWhat();
  status_ = std::move(other.status_);
  what_ = std::move(other.what_);
  return *this;
}

BadStatusOrAccess::BadStatusOrAccess(BadStatusOrAccess&& other)
    : status_(std::move(other.status_)) {}

const char* BadStatusOrAccess::what() const noexcept {
  InitWhat();
  return what_.c_str();
}

const util::Status& BadStatusOrAccess::status() const { return status_; }

void BadStatusOrAccess::InitWhat() const {
  std::call_once(init_what_, [this] {
    what_ = "Bad StatusOr access: " + status_.ToString();
  });
}

namespace internal_statusor {

void Helper::HandleInvalidStatusCtorArg(util::Status* status) {
  const char* kMessage =
      "An OK status is not a valid constructor argument to StatusOr<T>";
  *status = util::InternalError(kMessage);
}

void Helper::Crash(const util::Status& status) {
  LOG(FATAL) << "Attempting to fetch value instead of handling error "
             << status.ToString();
}

void ThrowBadStatusOrAccess(util::Status status) {
  LOG(FATAL) << "Attempting to fetch value instead of handling error "
             << status.ToString();
}

}  // namespace internal_statusor
}  // namespace util