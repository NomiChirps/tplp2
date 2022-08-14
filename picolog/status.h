#ifndef PICOLOG_STATUS_H_
#define PICOLOG_STATUS_H_

#include <ostream>
#include <string>
#include <cassert>

#include "picolog/status_internal.h"

namespace util {

// util::StatusCode
//
// An `util::StatusCode` is an enumerated type indicating either no error ("OK")
// or an error condition. In most cases, an `util::Status` indicates a
// recoverable error, and the purpose of signalling an error is to indicate what
// action to take in response to that error. These error codes map to the proto
// RPC error codes indicated in https://cloud.google.com/apis/design/errors.
//
// The errors listed below are the canonical errors associated with
// `util::Status` and are used throughout the codebase. As a result, these
// error codes are somewhat generic.
//
// In general, try to return the most specific error that applies if more than
// one error may pertain. For example, prefer `kOutOfRange` over
// `kFailedPrecondition` if both codes apply. Similarly prefer `kNotFound` or
// `kAlreadyExists` over `kFailedPrecondition`.
enum class StatusCode : int {
  // StatusCode::kOk
  //
  // kOK does not indicate an error; this value is returned on
  // success. It is typical to check for this value before proceeding on any
  // given call across an API or RPC boundary. To check this value, use the
  // `Status::ok()` member function rather than inspecting the raw code.
  kOk = 0,

  // StatusCode::kCancelled
  //
  // kCancelled indicates the operation was cancelled,
  // typically by the caller.
  kCancelled = 1,

  // StatusCode::kUnknown
  //
  // kUnknown indicates an unknown error occurred. In
  // general, more specific errors should be raised, if possible. Errors raised
  // by APIs that do not return enough error information may be converted to
  // this error.
  kUnknown = 2,

  // StatusCode::kInvalidArgument
  //
  // kInvalidArgument indicates the caller
  // specified an invalid argument, such as a malformed filename. Note that use
  // of such errors should be narrowly limited to indicate the invalid nature of
  // the arguments themselves. Errors with validly formed arguments that may
  // cause errors with the state of the receiving system should be denoted with
  // `kFailedPrecondition` instead.
  kInvalidArgument = 3,

  // StatusCode::kDeadlineExceeded
  //
  // kDeadlineExceeded indicates a deadline
  // expired before the operation could complete. For operations that may change
  // state within a system, this error may be returned even if the operation has
  // completed successfully. For example, a successful response from a server
  // could have been delayed long enough for the deadline to expire.
  kDeadlineExceeded = 4,

  // StatusCode::kNotFound
  //
  // kNotFound indicates some requested entity (such as
  // a file or directory) was not found.
  //
  // `kNotFound` is useful if a request should be denied for an entire class of
  // users, such as during a gradual feature rollout or undocumented allow list.
  // If a request should be denied for specific sets of users, such as through
  // user-based access control, use `kPermissionDenied` instead.
  kNotFound = 5,

  // StatusCode::kAlreadyExists
  //
  // kAlreadyExists indicates that the entity a
  // caller attempted to create (such as a file or directory) is already
  // present.
  kAlreadyExists = 6,

  // StatusCode::kPermissionDenied
  //
  // kPermissionDenied indicates that the caller
  // does not have permission to execute the specified operation. Note that this
  // error is different than an error due to an *un*authenticated user. This
  // error code does not imply the request is valid or the requested entity
  // exists or satisfies any other pre-conditions.
  //
  // `kPermissionDenied` must not be used for rejections caused by exhausting
  // some resource. Instead, use `kResourceExhausted` for those errors.
  // `kPermissionDenied` must not be used if the caller cannot be identified.
  // Instead, use `kUnauthenticated` for those errors.
  kPermissionDenied = 7,

  // StatusCode::kResourceExhausted
  //
  // kResourceExhausted indicates some resource
  // has been exhausted, perhaps a per-user quota, or perhaps the entire file
  // system is out of space.
  kResourceExhausted = 8,

  // StatusCode::kFailedPrecondition
  //
  // kFailedPrecondition indicates that the
  // operation was rejected because the system is not in a state required for
  // the operation's execution. For example, a directory to be deleted may be
  // non-empty, an "rmdir" operation is applied to a non-directory, etc.
  //
  // Some guidelines that may help a service implementer in deciding between
  // `kFailedPrecondition`, `kAborted`, and `kUnavailable`:
  //
  //  (a) Use `kUnavailable` if the client can retry just the failing call.
  //  (b) Use `kAborted` if the client should retry at a higher transaction
  //      level (such as when a client-specified test-and-set fails, indicating
  //      the client should restart a read-modify-write sequence).
  //  (c) Use `kFailedPrecondition` if the client should not retry until
  //      the system state has been explicitly fixed. For example, if a "rmdir"
  //      fails because the directory is non-empty, `kFailedPrecondition`
  //      should be returned since the client should not retry unless
  //      the files are deleted from the directory.
  kFailedPrecondition = 9,

  // StatusCode::kAborted
  //
  // kAborted indicates the operation was aborted,
  // typically due to a concurrency issue such as a sequencer check failure or a
  // failed transaction.
  //
  // See the guidelines above for deciding between `kFailedPrecondition`,
  // `kAborted`, and `kUnavailable`.
  kAborted = 10,

  // StatusCode::kOutOfRange
  //
  // kOutOfRange indicates the operation was
  // attempted past the valid range, such as seeking or reading past an
  // end-of-file.
  //
  // Unlike `kInvalidArgument`, this error indicates a problem that may
  // be fixed if the system state changes. For example, a 32-bit file
  // system will generate `kInvalidArgument` if asked to read at an
  // offset that is not in the range [0,2^32-1], but it will generate
  // `kOutOfRange` if asked to read from an offset past the current
  // file size.
  //
  // There is a fair bit of overlap between `kFailedPrecondition` and
  // `kOutOfRange`.  We recommend using `kOutOfRange` (the more specific
  // error) when it applies so that callers who are iterating through
  // a space can easily look for an `kOutOfRange` error to detect when
  // they are done.
  kOutOfRange = 11,

  // StatusCode::kUnimplemented
  //
  // kUnimplemented indicates the operation is not
  // implemented or supported in this service. In this case, the operation
  // should not be re-attempted.
  kUnimplemented = 12,

  // StatusCode::kInternal
  //
  // kInternal indicates an internal error has occurred
  // and some invariants expected by the underlying system have not been
  // satisfied. This error code is reserved for serious errors.
  kInternal = 13,

  // StatusCode::kUnavailable
  //
  // kUnavailable indicates the service is currently
  // unavailable and that this is most likely a transient condition. An error
  // such as this can be corrected by retrying with a backoff scheme. Note that
  // it is not always safe to retry non-idempotent operations.
  //
  // See the guidelines above for deciding between `kFailedPrecondition`,
  // `kAborted`, and `kUnavailable`.
  kUnavailable = 14,

  // StatusCode::kDataLoss
  //
  // kDataLoss indicates that unrecoverable data loss or
  // corruption has occurred. As this error is serious, proper alerting should
  // be attached to errors such as this.
  kDataLoss = 15,

  // StatusCode::kUnauthenticated
  //
  // kUnauthenticated indicates that the request
  // does not have valid authentication credentials for the operation. Correct
  // the authentication and try again.
  kUnauthenticated = 16,

  // StatusCode::DoNotUseReservedForFutureExpansionUseDefaultInSwitchInstead_
  //
  // NOTE: this error code entry should not be used and you should not rely on
  // its value, which may change.
  //
  // The purpose of this enumerated value is to force people who handle status
  // codes with `switch()` statements to *not* simply enumerate all possible
  // values, but instead provide a "default:" case. Providing such a default
  // case ensures that code will compile when new codes are added.
  kDoNotUseReservedForFutureExpansionUseDefaultInSwitchInstead_ = 20
};

// StatusCodeToString()
//
// Returns the name for the status code, or "" if it is an unknown value.
std::string StatusCodeToString(StatusCode code);

// operator<<
//
// Streams StatusCodeToString(code) to `os`.
std::ostream& operator<<(std::ostream& os, StatusCode code);

// The `util::Status` class is generally used to gracefully handle errors
// across API boundaries (and in particular across RPC boundaries). Some of
// these errors may be recoverable, but others may not. Most
// functions which can produce a recoverable error should be designed to return
// either an `util::Status` (or the similar `util::StatusOr<T>`, which holds
// either an object of type `T` or an error).
//
// API developers should construct their functions to return `util::OkStatus()`
// upon success, or an `util::StatusCode` upon another type of error (e.g
// an `util::StatusCode::kInvalidArgument` error). The API provides convenience
// functions to construct each status code.
//
// Example:
//
// util::Status myFunction(std::string_view fname, ...) {
//   ...
//   // encounter error
//   if (error condition) {
//     // Construct an util::StatusCode::kInvalidArgument error
//     return util::InvalidArgumentError("bad mode");
//   }
//   // else, return OK
//   return util::OkStatus();
// }
//
// Users handling status error codes should prefer checking for an OK status
// using the `ok()` member function. Handling multiple error codes may justify
// use of switch statement, but only check for error codes you know how to
// handle; do not try to exhaustively match against all canonical error codes.
// Errors that cannot be handled should be logged and/or propagated for higher
// levels to deal with. If you do use a switch statement, make sure that you
// also provide a `default:` switch case, so that code does not break as other
// canonical codes are added to the API.
//
// Example:
//
//   util::Status result = DoSomething();
//   if (!result.ok()) {
//     LOG(ERROR) << result;
//   }
//
//   // Provide a default if switching on multiple error codes
//   switch (result.code()) {
//     // The user hasn't authenticated. Ask them to reauth
//     case util::StatusCode::kUnauthenticated:
//       DoReAuth();
//       break;
//     // The user does not have permission. Log an error.
//     case util::StatusCode::kPermissionDenied:
//       LOG(ERROR) << result;
//       break;
//     // Propagate the error otherwise.
//     default:
//       return true;
//   }
//
// For documentation see https://abseil.io/docs/cpp/guides/status.
//
// Returned Status objects may not be ignored. status_internal.h has a forward
class [[nodiscard]] Status final {
 public:
  // Constructors

  // This default constructor creates an OK status with no message or payload.
  // Avoid this constructor and prefer explicit construction of an OK status
  // with `util::OkStatus()`.
  Status();

  // Creates a status in the canonical error space with the specified
  // `util::StatusCode` and error message.  If `code == util::StatusCode::kOk`,  // NOLINT
  // `msg` is ignored and an object identical to an OK status is constructed.
  //
  // The `msg` string must be in UTF-8. The implementation may complain (e.g.,  // NOLINT
  // by printing a warning) if it is not.
  Status(util::StatusCode code, std::string_view msg);

  // XXX: Copy not supported because ARMv6-M lacks synchronization primitives
  //      for the reference-counting implementation.
  Status(const Status&) = delete;
  Status& operator=(const Status& x) = delete;

  // Move operators

  // The moved-from state is valid but unspecified.
  Status(Status&&) noexcept;
  Status& operator=(Status&&);

  ~Status();

  // Status::Update()
  //
  // Updates the existing status with `new_status` provided that `this->ok()`.
  // If the existing status already contains a non-OK error, this update has no
  // effect and preserves the current data. Note that this behavior may change
  // in the future to augment a current non-ok status with additional
  // information about `new_status`.
  //
  // `Update()` provides a convenient way of keeping track of the first error
  // encountered.
  //
  // Example:
  //   // Instead of "if (overall_status.ok()) overall_status = new_status"
  //   overall_status.Update(new_status);
  //
  void Update(Status&& new_status);

  // Status::ok()
  //
  // Returns `true` if `this->code()` == `util::StatusCode::kOk`,
  // indicating the absence of an error.
  // Prefer checking for an OK status using this member function.
  [[nodiscard]] bool ok() const;

  // Status::code()
  //
  // Returns the canonical error code of type `util::StatusCode` of this status.
  util::StatusCode code() const;

  // Status::raw_code()
  //
  // Returns a raw (canonical) error code corresponding to the enum value of
  // `google.rpc.Code` definitions within
  // https://github.com/googleapis/googleapis/blob/master/google/rpc/code.proto.
  // These values could be out of the range of canonical `util::StatusCode`
  // enum values.
  //
  // NOTE: This function should only be called when converting to an associated
  // wire format. Use `Status::code()` for error handling.
  int raw_code() const;

  // Status::message()
  //
  // Returns the error message associated with this error code, if available.
  // Note that this message rarely describes the error code.  It is not unusual
  // for the error message to be the empty string. As a result, prefer
  // `operator<<` or `Status::ToString()` for debug logging.
  std::string_view message() const;

  friend bool operator==(const Status&, const Status&);
  friend bool operator!=(const Status&, const Status&);

  // Status::ToString()
  //
  // The printed code name and the message are generally substrings of the
  // result.
  std::string ToString() const;

  // Status::IgnoreError()
  //
  // Ignores any errors. This method does nothing except potentially suppress
  // complaints from any tools that are checking that errors are not dropped on
  // the floor.
  void IgnoreError() const;

  // swap()
  //
  // Swap the contents of one status with another.
  friend void swap(Status& a, Status& b);

 private:
  friend Status CancelledError();

  // Creates a status in the canonical error space with the specified
  // code, and an empty error message.
  explicit Status(util::StatusCode code);

  static bool EqualsSlow(const util::Status& a, const util::Status& b);

  static constexpr char kMovedFromString[] =
      "Status accessed after move.";

  static const std::string* EmptyString();
  static const std::string* MovedFromString();

  // Returns whether rep contains an inlined representation.
  // See rep_ for details.
  static bool IsInlined(uintptr_t rep);

  // Indicates whether this Status was the rhs of a move operation. See rep_
  // for details.
  static bool IsMovedFrom(uintptr_t rep);
  static uintptr_t MovedFromRep();

  // Convert between error::Code and the inlined uintptr_t representation used
  // by rep_. See rep_ for details.
  static uintptr_t CodeToInlinedRep(util::StatusCode code);
  static util::StatusCode InlinedRepToCode(uintptr_t rep);

  // Converts between StatusRep* and the external uintptr_t representation used
  // by rep_. See rep_ for details.
  static uintptr_t PointerToRep(status_internal::StatusRep* r);
  static status_internal::StatusRep* RepToPointer(uintptr_t r);

  std::string ToStringSlow() const;

  // Status supports two different representations.
  //  - When the low bit is off it is an inlined representation.
  //    It uses the canonical error space, no message or payload.
  //    The error code is (rep_ >> 2).
  //    The (rep_ & 2) bit is the "moved from" indicator, used in IsMovedFrom().
  //  - When the low bit is on it is an external representation.
  //    In this case all the data comes from a heap allocated Rep object.
  //    (rep_ - 1) is a status_internal::StatusRep* pointer to that structure.
  uintptr_t rep_;
};

// OkStatus()
//
// Returns an OK status, equivalent to a default constructed instance. Prefer
// usage of `util::OkStatus()` when constructing such an OK status.
Status OkStatus();

// operator<<()
//
// Prints a human-readable representation of `x` to `os`.
std::ostream& operator<<(std::ostream& os, const Status& x);

// IsAborted()
// IsAlreadyExists()
// IsCancelled()
// IsDataLoss()
// IsDeadlineExceeded()
// IsFailedPrecondition()
// IsInternal()
// IsInvalidArgument()
// IsNotFound()
// IsOutOfRange()
// IsPermissionDenied()
// IsResourceExhausted()
// IsUnauthenticated()
// IsUnavailable()
// IsUnimplemented()
// IsUnknown()
//
// These convenience functions return `true` if a given status matches the
// `util::StatusCode` error code of its associated function.
[[nodiscard]] bool IsAborted(const Status& status);
[[nodiscard]] bool IsAlreadyExists(const Status& status);
[[nodiscard]] bool IsCancelled(const Status& status);
[[nodiscard]] bool IsDataLoss(const Status& status);
[[nodiscard]] bool IsDeadlineExceeded(const Status& status);
[[nodiscard]] bool IsFailedPrecondition(const Status& status);
[[nodiscard]] bool IsInternal(const Status& status);
[[nodiscard]] bool IsInvalidArgument(const Status& status);
[[nodiscard]] bool IsNotFound(const Status& status);
[[nodiscard]] bool IsOutOfRange(const Status& status);
[[nodiscard]] bool IsPermissionDenied(const Status& status);
[[nodiscard]] bool IsResourceExhausted(const Status& status);
[[nodiscard]] bool IsUnauthenticated(const Status& status);
[[nodiscard]] bool IsUnavailable(const Status& status);
[[nodiscard]] bool IsUnimplemented(const Status& status);
[[nodiscard]] bool IsUnknown(const Status& status);

// AbortedError()
// AlreadyExistsError()
// CancelledError()
// DataLossError()
// DeadlineExceededError()
// FailedPreconditionError()
// InternalError()
// InvalidArgumentError()
// NotFoundError()
// OutOfRangeError()
// PermissionDeniedError()
// ResourceExhaustedError()
// UnauthenticatedError()
// UnavailableError()
// UnimplementedError()
// UnknownError()
//
// These convenience functions create an `util::Status` object with an error
// code as indicated by the associated function name, using the error message
// passed in `message`.
Status AbortedError(std::string_view message);
Status AlreadyExistsError(std::string_view message);
Status CancelledError(std::string_view message);
Status DataLossError(std::string_view message);
Status DeadlineExceededError(std::string_view message);
Status FailedPreconditionError(std::string_view message);
Status InternalError(std::string_view message);
Status InvalidArgumentError(std::string_view message);
Status NotFoundError(std::string_view message);
Status OutOfRangeError(std::string_view message);
Status PermissionDeniedError(std::string_view message);
Status ResourceExhaustedError(std::string_view message);
Status UnauthenticatedError(std::string_view message);
Status UnavailableError(std::string_view message);
Status UnimplementedError(std::string_view message);
Status UnknownError(std::string_view message);

//------------------------------------------------------------------------------
// Implementation details follow
//------------------------------------------------------------------------------

inline Status::Status() : rep_(CodeToInlinedRep(util::StatusCode::kOk)) {}

inline Status::Status(util::StatusCode code) : rep_(CodeToInlinedRep(code)) {}

inline Status::Status(Status&& x) noexcept : rep_(x.rep_) {
  x.rep_ = MovedFromRep();
}

inline Status& Status::operator=(Status&& x) {
  uintptr_t old_rep = rep_;
  if (x.rep_ != old_rep) {
    rep_ = x.rep_;
    x.rep_ = MovedFromRep();
  }
  return *this;
}

inline void Status::Update(Status&& new_status) {
  if (ok()) {
    *this = std::move(new_status);
  }
}

inline Status::~Status() {
  if (!IsInlined(rep_)) {
    delete RepToPointer(rep_);
  }
}

inline bool Status::ok() const {
  return rep_ == CodeToInlinedRep(util::StatusCode::kOk);
}

inline std::string_view Status::message() const {
  return !IsInlined(rep_)
             ? RepToPointer(rep_)->message
             : (IsMovedFrom(rep_) ? std::string_view(kMovedFromString)
                                  : std::string_view());
}

inline bool operator==(const Status& lhs, const Status& rhs) {
  return lhs.rep_ == rhs.rep_ || Status::EqualsSlow(lhs, rhs);
}

inline bool operator!=(const Status& lhs, const Status& rhs) {
  return !(lhs == rhs);
}

inline std::string Status::ToString() const {
  return ok() ? "OK" : ToStringSlow();
}

inline void Status::IgnoreError() const {
  // no-op
}

inline void swap(util::Status& a, util::Status& b) {
  using std::swap;
  swap(a.rep_, b.rep_);
}

inline bool Status::IsInlined(uintptr_t rep) { return (rep & 1) == 0; }


inline bool Status::IsMovedFrom(uintptr_t rep) {
  return IsInlined(rep) && (rep & 2) != 0;
}

inline uintptr_t Status::MovedFromRep() {
  return CodeToInlinedRep(util::StatusCode::kInternal) | 2;
}

inline uintptr_t Status::CodeToInlinedRep(util::StatusCode code) {
  return static_cast<uintptr_t>(code) << 2;
}

inline util::StatusCode Status::InlinedRepToCode(uintptr_t rep) {
  assert(IsInlined(rep));
  return static_cast<util::StatusCode>(rep >> 2);
}

inline status_internal::StatusRep* Status::RepToPointer(uintptr_t rep) {
  assert(!IsInlined(rep));
  return reinterpret_cast<status_internal::StatusRep*>(rep - 1);
}

inline uintptr_t Status::PointerToRep(status_internal::StatusRep* rep) {
  return reinterpret_cast<uintptr_t>(rep) + 1;
}

inline Status OkStatus() { return Status(); }

// Creates a `Status` object with the `util::StatusCode::kCancelled` error code
// and an empty message. It is provided only for efficiency, given that
// message-less kCancelled errors are common in the infrastructure.
inline Status CancelledError() { return Status(util::StatusCode::kCancelled); }


}  // namespace util

#endif  // PICOLOG_STATUS_H_