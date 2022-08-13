#include "picolog/status.h"

#include <cassert>
#include <sstream>

#include "picolog/status_internal.h"

namespace util {
std::string StatusCodeToString(StatusCode code) {
  switch (code) {
    case StatusCode::kOk:
      return "OK";
    case StatusCode::kCancelled:
      return "CANCELLED";
    case StatusCode::kUnknown:
      return "UNKNOWN";
    case StatusCode::kInvalidArgument:
      return "INVALID_ARGUMENT";
    case StatusCode::kDeadlineExceeded:
      return "DEADLINE_EXCEEDED";
    case StatusCode::kNotFound:
      return "NOT_FOUND";
    case StatusCode::kAlreadyExists:
      return "ALREADY_EXISTS";
    case StatusCode::kPermissionDenied:
      return "PERMISSION_DENIED";
    case StatusCode::kUnauthenticated:
      return "UNAUTHENTICATED";
    case StatusCode::kResourceExhausted:
      return "RESOURCE_EXHAUSTED";
    case StatusCode::kFailedPrecondition:
      return "FAILED_PRECONDITION";
    case StatusCode::kAborted:
      return "ABORTED";
    case StatusCode::kOutOfRange:
      return "OUT_OF_RANGE";
    case StatusCode::kUnimplemented:
      return "UNIMPLEMENTED";
    case StatusCode::kInternal:
      return "INTERNAL";
    case StatusCode::kUnavailable:
      return "UNAVAILABLE";
    case StatusCode::kDataLoss:
      return "DATA_LOSS";
    default:
      return "";
  }
}

std::ostream& operator<<(std::ostream& os, StatusCode code) {
  return os << StatusCodeToString(code);
}

namespace status_internal {

// Convert canonical code to a value known to this binary.
util::StatusCode MapToLocalCode(int value) {
  util::StatusCode code = static_cast<util::StatusCode>(value);
  switch (code) {
    case util::StatusCode::kOk:
    case util::StatusCode::kCancelled:
    case util::StatusCode::kUnknown:
    case util::StatusCode::kInvalidArgument:
    case util::StatusCode::kDeadlineExceeded:
    case util::StatusCode::kNotFound:
    case util::StatusCode::kAlreadyExists:
    case util::StatusCode::kPermissionDenied:
    case util::StatusCode::kResourceExhausted:
    case util::StatusCode::kFailedPrecondition:
    case util::StatusCode::kAborted:
    case util::StatusCode::kOutOfRange:
    case util::StatusCode::kUnimplemented:
    case util::StatusCode::kInternal:
    case util::StatusCode::kUnavailable:
    case util::StatusCode::kDataLoss:
    case util::StatusCode::kUnauthenticated:
      return code;
    default:
      return util::StatusCode::kUnknown;
  }
}
}  // namespace status_internal

const std::string* Status::EmptyString() {
  static union EmptyString {
    std::string str;
    ~EmptyString() {}
  } empty = {{}};
  return &empty.str;
}

const std::string* Status::MovedFromString() {
  static std::string* moved_from_string = new std::string(kMovedFromString);
  return moved_from_string;
}

void Status::UnrefNonInlined(uintptr_t rep) {
  status_internal::StatusRep* r = RepToPointer(rep);
  // Fast path: if ref==1, there is no need for a RefCountDec (since
  // this is the only reference and therefore no other thread is
  // allowed to be mucking with r).
  if (r->ref.load(std::memory_order_acquire) == 1 ||
      r->ref.fetch_sub(1, std::memory_order_acq_rel) - 1 == 0) {
    delete r;
  }
}

Status::Status(util::StatusCode code, std::string_view msg)
    : rep_(CodeToInlinedRep(code)) {
  if (code != util::StatusCode::kOk && !msg.empty()) {
    rep_ = PointerToRep(new status_internal::StatusRep(code, msg));
  }
}

int Status::raw_code() const {
  if (IsInlined(rep_)) {
    return static_cast<int>(InlinedRepToCode(rep_));
  }
  status_internal::StatusRep* rep = RepToPointer(rep_);
  return static_cast<int>(rep->code);
}

util::StatusCode Status::code() const {
  return status_internal::MapToLocalCode(raw_code());
}

bool Status::EqualsSlow(const util::Status& a, const util::Status& b) {
  if (IsInlined(a.rep_) != IsInlined(b.rep_)) return false;
  if (a.message() != b.message()) return false;
  if (a.raw_code() != b.raw_code()) return false;

  return true;
}

std::string Status::ToStringSlow() const {
  // TODO: this was StrAppend
  std::ostringstream text;
  text << util::StatusCodeToString(code()) << ": " << message();

  return text.str();
}

std::ostream& operator<<(std::ostream& os, const Status& x) {
  os << x.ToString();
  return os;
}

Status AbortedError(std::string_view message) {
  return Status(util::StatusCode::kAborted, message);
}

Status AlreadyExistsError(std::string_view message) {
  return Status(util::StatusCode::kAlreadyExists, message);
}

Status CancelledError(std::string_view message) {
  return Status(util::StatusCode::kCancelled, message);
}

Status DataLossError(std::string_view message) {
  return Status(util::StatusCode::kDataLoss, message);
}

Status DeadlineExceededError(std::string_view message) {
  return Status(util::StatusCode::kDeadlineExceeded, message);
}

Status FailedPreconditionError(std::string_view message) {
  return Status(util::StatusCode::kFailedPrecondition, message);
}

Status InternalError(std::string_view message) {
  return Status(util::StatusCode::kInternal, message);
}

Status InvalidArgumentError(std::string_view message) {
  return Status(util::StatusCode::kInvalidArgument, message);
}

Status NotFoundError(std::string_view message) {
  return Status(util::StatusCode::kNotFound, message);
}

Status OutOfRangeError(std::string_view message) {
  return Status(util::StatusCode::kOutOfRange, message);
}

Status PermissionDeniedError(std::string_view message) {
  return Status(util::StatusCode::kPermissionDenied, message);
}

Status ResourceExhaustedError(std::string_view message) {
  return Status(util::StatusCode::kResourceExhausted, message);
}

Status UnauthenticatedError(std::string_view message) {
  return Status(util::StatusCode::kUnauthenticated, message);
}

Status UnavailableError(std::string_view message) {
  return Status(util::StatusCode::kUnavailable, message);
}

Status UnimplementedError(std::string_view message) {
  return Status(util::StatusCode::kUnimplemented, message);
}

Status UnknownError(std::string_view message) {
  return Status(util::StatusCode::kUnknown, message);
}

bool IsAborted(const Status& status) {
  return status.code() == util::StatusCode::kAborted;
}

bool IsAlreadyExists(const Status& status) {
  return status.code() == util::StatusCode::kAlreadyExists;
}

bool IsCancelled(const Status& status) {
  return status.code() == util::StatusCode::kCancelled;
}

bool IsDataLoss(const Status& status) {
  return status.code() == util::StatusCode::kDataLoss;
}

bool IsDeadlineExceeded(const Status& status) {
  return status.code() == util::StatusCode::kDeadlineExceeded;
}

bool IsFailedPrecondition(const Status& status) {
  return status.code() == util::StatusCode::kFailedPrecondition;
}

bool IsInternal(const Status& status) {
  return status.code() == util::StatusCode::kInternal;
}

bool IsInvalidArgument(const Status& status) {
  return status.code() == util::StatusCode::kInvalidArgument;
}

bool IsNotFound(const Status& status) {
  return status.code() == util::StatusCode::kNotFound;
}

bool IsOutOfRange(const Status& status) {
  return status.code() == util::StatusCode::kOutOfRange;
}

bool IsPermissionDenied(const Status& status) {
  return status.code() == util::StatusCode::kPermissionDenied;
}

bool IsResourceExhausted(const Status& status) {
  return status.code() == util::StatusCode::kResourceExhausted;
}

bool IsUnauthenticated(const Status& status) {
  return status.code() == util::StatusCode::kUnauthenticated;
}

bool IsUnavailable(const Status& status) {
  return status.code() == util::StatusCode::kUnavailable;
}

bool IsUnimplemented(const Status& status) {
  return status.code() == util::StatusCode::kUnimplemented;
}

bool IsUnknown(const Status& status) {
  return status.code() == util::StatusCode::kUnknown;
}

namespace status_internal {

std::string* MakeCheckFailString(const util::Status* status,
                                 const char* prefix) {
  // TODO: this was StrCat
  std::ostringstream text;
  text << prefix << " (" << status->ToString() << ")";
  return new std::string(text.str());
}

}  // namespace status_internal
}  // namespace util