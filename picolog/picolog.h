#ifndef PICOLOG_LOGGING_H_
#define PICOLOG_LOGGING_H_

#include <array>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <ostream>
#include <sstream>

#include "picolog/status.h"
#include "picolog/vlog_is_on.h"

// PICOLOG_VMODULE allows selectively enabling VLOG(n) lines at compile time.
// The format is "module=n" pairs, where module is the bare name of the .cc/.h
// file to enable verbose logging for, and n is the verbose log level to enable.
// This configuration string must be double-quoted.
//
// For example:
//   -DPICOLOG_VMODULE=\"foo=4,bar=0\"
// will cause foo.h and foo.cc to include all VLOG(n) lines where n is 4 or
// less. bar.h and bar.cc will not compile any VLOG statements at all (which is
// the default).
#ifndef PICOLOG_VMODULE
#define PICOLOG_VMODULE ""
#endif

// Yes, large parts of this library have been ripped off verbatim from glog.
namespace picolog {

// Call before doing anything else with this library. Note that this does not
// initialize any USB/UART/etc output hardware; you should do that separately.
void InitLogging();

// Run this function as a task under FreeRTOS in order for log messages to
// actually go somewhere.
// TODO: support logging before the scheduler starts
[[noreturn]] void BackgroundTask(void* ignored);

// Returns true if the system is in the midst of crashing due to a fatal error.
bool IsDying();

///////////////////////////////////////////////////////////////////////////////

typedef int LogSeverity;

constexpr int PICOLOG_INFO = 0, PICOLOG_WARNING = 1, PICOLOG_ERROR = 2,
              PICOLOG_FATAL = 3, NUM_SEVERITIES = 4;
constexpr int INFO = PICOLOG_INFO, WARNING = PICOLOG_WARNING,
              ERROR = PICOLOG_ERROR, FATAL = PICOLOG_FATAL;

// We're not using the RTC (yet?) so this only has the microseconds since boot.
struct LogMessageTime {
  LogMessageTime() : usecs_since_boot_(0) {}
  LogMessageTime(std::time_t usecs_since_boot)
      : usecs_since_boot_(usecs_since_boot) {}

  int hour() const;
  int min() const;
  int sec() const;
  int usec() const;

 private:
  std::time_t usecs_since_boot_;
};

#define PICOLOG_PREDICT_BRANCH_NOT_TAKEN(x) (__builtin_expect(x, 0))
#define PICOLOG_PREDICT_TRUE(x) (__builtin_expect(!!(x), 1))
#define PICOLOG_PREDICT_FALSE(x) (__builtin_expect(x, 0))

// A container for a string pointer which can be evaluated to a bool -
// true iff the pointer is NULL.
struct CheckOpString {
  CheckOpString(std::string* str) : str_(str) {}
  // No destructor: if str_ is non-NULL, we're about to LOG(FATAL),
  // so there's no point in cleaning up str_.
  operator bool() const {
    return PICOLOG_PREDICT_BRANCH_NOT_TAKEN(str_ != NULL);
  }
  std::string* str_;
};

// Function is overloaded for integral types to allow static const
// integrals declared in classes and not defined to be used as arguments to
// CHECK* macros. It's not encouraged though.
template <class T>
inline const T& GetReferenceableValue(const T& t) {
  return t;
}
inline char GetReferenceableValue(char t) { return t; }
inline unsigned char GetReferenceableValue(unsigned char t) { return t; }
inline signed char GetReferenceableValue(signed char t) { return t; }
inline short GetReferenceableValue(short t) { return t; }
inline unsigned short GetReferenceableValue(unsigned short t) { return t; }
inline int GetReferenceableValue(int t) { return t; }
inline unsigned int GetReferenceableValue(unsigned int t) { return t; }
inline long GetReferenceableValue(long t) { return t; }
inline unsigned long GetReferenceableValue(unsigned long t) { return t; }
#if __cplusplus >= 201103L
inline long long GetReferenceableValue(long long t) { return t; }
inline unsigned long long GetReferenceableValue(unsigned long long t) {
  return t;
}
#endif

// This is a dummy class to define the following operator.
struct DummyClassToDefineOperator {};

// Define global operator<< to declare using ::operator<<.
// This declaration will allow use to use CHECK macros for user
// defined classes which have operator<< (e.g., stl_logging.h).
inline std::ostream& operator<<(std::ostream& out,
                                const DummyClassToDefineOperator&) {
  return out;
}

// This formats a value for a failing CHECK_XX statement.  Ordinarily,
// it uses the definition for operator<<, with a few special cases below.
template <typename T>
inline void MakeCheckOpValueString(std::ostream* os, const T& v) {
  (*os) << v;
}

// Overrides for char types provide readable values for unprintable
// characters.
template <>
void MakeCheckOpValueString(std::ostream* os, const char& v);
template <>
void MakeCheckOpValueString(std::ostream* os, const signed char& v);
template <>
void MakeCheckOpValueString(std::ostream* os, const unsigned char& v);

// Provide printable value for nullptr_t
template <>
void MakeCheckOpValueString(std::ostream* os, const std::nullptr_t& v);

// Build the error message string. Specify no inlining for code size.
template <typename T1, typename T2>
[[gnu::noinline]] std::string* MakeCheckOpString(const T1& v1, const T2& v2,
                                                 const char* exprtext);

// LogMessage::LogStream is a std::ostream backed by this streambuf.
// This class ignores overflow and leaves two bytes at the end of the
// buffer to allow for a '\n' and '\0'.
class LogStreamBuf : public std::streambuf {
 public:
  // REQUIREMENTS: "len" must be >= 2 to account for the '\n' and '\0'.
  LogStreamBuf(char* buf, int len) { setp(buf, buf + len - 2); }

  // This effectively ignores overflow.
  int_type overflow(int_type ch) { return ch; }

  // Legacy public ostrstream method.
  size_t pcount() const { return static_cast<size_t>(pptr() - pbase()); }
  char* pbase() const { return std::streambuf::pbase(); }
};

class LogMessage {
 public:
  class LogStream : public std::ostream {
   public:
    LogStream(char* buf, int len) : std::ostream(NULL), streambuf_(buf, len) {
      rdbuf(&streambuf_);
    }

    // Legacy std::streambuf methods.
    size_t pcount() const { return streambuf_.pcount(); }
    char* pbase() const { return streambuf_.pbase(); }
    char* str() const { return pbase(); }

   private:
    LogStream(const LogStream&);
    LogStream& operator=(const LogStream&);
    LogStreamBuf streambuf_;
  };

  // Implicitly at severity INFO to save a little space at the call site,
  // apparently. Thanks for the idea, glog.
  LogMessage(const char* file, int line);
  LogMessage(const char* file, int line, LogSeverity severity);
  // A special constructor used for check failures
  LogMessage(const char* file, int line, const CheckOpString& result);

  ~LogMessage();

  // An arbitrary limit on the length of a single log message.  This
  // is so that streaming can be done more efficiently.
  static const size_t kMaxLogMessageLen;

  std::ostream& stream();

  struct LogMessageData;

 protected:
  void Flush();
  // Perform the FATAL crash, i.e. prevent the current thread from proceeding.
  // The actual systemwide crash will happen from the background task after it
  // has logged the message.
  [[noreturn]] static void Fail();

 private:
  void Init(const char* file, int line, LogSeverity severity);

  // We keep the data in a separate struct so that each instance of
  // LogMessage uses less stack space.
  LogMessageData* data_;

  LogMessage(const LogMessage&) = delete;
  void operator=(const LogMessage&) = delete;

  friend void BackgroundTask(void*);
};

class LogMessageFatal : public LogMessage {
 public:
  LogMessageFatal(const char* file, int line);
  LogMessageFatal(const char* file, int line, const CheckOpString& result);
  [[noreturn]] ~LogMessageFatal();
};

class NullStream : public LogMessage::LogStream {
 public:
  // Initialize the LogStream so the messages can be written somewhere
  // (they'll never be actually displayed). This will be needed if a
  // NullStream& is implicitly converted to LogStream&, in which case
  // the overloaded NullStream::operator<< will not be invoked.
  NullStream() : LogMessage::LogStream(message_buffer_, 1) {}
  NullStream(const char* /*file*/, int /*line*/)
      : LogMessage::LogStream(message_buffer_, 1) {}
  NullStream(const char* /*file*/, int /*line*/,
             const CheckOpString& /*result*/)
      : LogMessage::LogStream(message_buffer_, 1) {}
  NullStream& stream() { return *this; }

 private:
  // A very short buffer for messages (which we discard anyway). This
  // will be needed if NullStream& converted to LogStream& (e.g. as a
  // result of a conditional expression).
  char message_buffer_[2];
};

// Do nothing. This operator is inline, allowing the message to be
// compiled away. The message will not be compiled away if we do
// something like (flag ? LOG(INFO) : LOG(ERROR)) << message; when
// SKIP_LOG=WARNING. In those cases, NullStream will be implicitly
// converted to LogStream and the message will be computed and then
// quietly discarded.
template <class T>
inline NullStream& operator<<(NullStream& str, const T&) {
  return str;
}

// Similar to NullStream, but aborts the program (without stack
// trace), like LogMessageFatal.
class NullStreamFatal : public NullStream {
 public:
  NullStreamFatal() {}
  NullStreamFatal(const char* file, int line) : NullStream(file, line) {}
  // TODO: uh, does _Exit actually do something on pico?
  [[noreturn]] ~NullStreamFatal() throw() { _Exit(EXIT_FAILURE); }
};

#if PICOLOG_STRIP_LOG == 0
#define PICOLOG_COMPACT_LOG_INFO \
  ::picolog::LogMessage(__FILE__, __LINE__).stream()
#else
#define PICOLOG_COMPACT_LOG_INFO ::picolog::NullStream()
#endif

#if PICOLOG_STRIP_LOG <= 1
#define PICOLOG_COMPACT_LOG_WARNING \
  ::picolog::LogMessage(__FILE__, __LINE__, ::picolog::PICOLOG_WARNING).stream()
#else
#define PICOLOG_COMPACT_LOG_WARNING ::picolog::NullStream()
#endif

#if PICOLOG_STRIP_LOG <= 2
#define PICOLOG_COMPACT_LOG_ERROR \
  ::picolog::LogMessage(__FILE__, __LINE__, ::picolog::PICOLOG_ERROR).stream()
#else
#define PICOLOG_COMPACT_LOG_ERROR ::picolog::NullStream()
#endif

#if PICOLOG_STRIP_LOG <= 3
#define PICOLOG_COMPACT_LOG_FATAL \
  ::picolog::LogMessageFatal(__FILE__, __LINE__).stream()
#else
#define PICOLOG_COMPACT_LOG_FATAL ::picolog::NullStreamFatal()
#endif

#define LOG(severity) PICOLOG_COMPACT_LOG_##severity

// Apparently the "voidification" gymnastics here are used to explicitly
// ignore values, avoiding compiler warnings about unused warnings. Thanks
// glog!
class LogMessageVoidify {
 public:
  LogMessageVoidify() {}
  void operator&(std::ostream&) {}
};

#define LOG_IF(severity, condition) \
  static_cast<void>(0),             \
      !(condition) ? (void)0 : ::picolog::LogMessageVoidify() & LOG(severity)

// A helper class for formatting "expr (V1 vs. V2)" in a CHECK_XX
// statement.  See MakeCheckOpString for sample usage.  Other
// approaches were considered: use of a template method (e.g.,
// base::BuildCheckOpString(exprtext, base::Print<T1>, &v1,
// base::Print<T2>, &v2), however this approach has complications
// related to volatile arguments and function-pointer arguments).
class CheckOpMessageBuilder {
 public:
  // Inserts "exprtext" and " (" to the stream.
  explicit CheckOpMessageBuilder(const char* exprtext);
  // Deletes "stream_".
  ~CheckOpMessageBuilder();
  // For inserting the first variable.
  std::ostream* ForVar1() { return stream_; }
  // For inserting the second variable (adds an intermediate " vs. ").
  std::ostream* ForVar2();
  // Get the result (inserts the closing ")").
  std::string* NewString();

 private:
  std::ostringstream* stream_;
};

template <typename T1, typename T2>
std::string* MakeCheckOpString(const T1& v1, const T2& v2,
                               const char* exprtext) {
  CheckOpMessageBuilder comb(exprtext);
  MakeCheckOpValueString(comb.ForVar1(), v1);
  MakeCheckOpValueString(comb.ForVar2(), v2);
  return comb.NewString();
}

// Helper functions for CHECK_OP macro.
// The (int, int) specialization works around the issue that the compiler
// will not instantiate the template version of the function on values of
// unnamed enum type - see comment below.
#define DEFINE_CHECK_OP_IMPL(name, op)                                   \
  template <typename T1, typename T2>                                    \
  inline std::string* name##Impl(const T1& v1, const T2& v2,             \
                                 const char* exprtext) {                 \
    if (PICOLOG_PREDICT_TRUE(v1 op v2))                                  \
      return NULL;                                                       \
    else                                                                 \
      return MakeCheckOpString(v1, v2, exprtext);                        \
  }                                                                      \
  inline std::string* name##Impl(int v1, int v2, const char* exprtext) { \
    return name##Impl<int, int>(v1, v2, exprtext);                       \
  }

// We use the full name Check_EQ, Check_NE, etc. in case the file including
// base/logging.h provides its own #defines for the simpler names EQ, NE, etc.
// This happens if, for example, those are used as token names in a
// yacc grammar.
DEFINE_CHECK_OP_IMPL(Check_EQ, ==)  // Compilation error with CHECK_EQ(NULL, x)?
DEFINE_CHECK_OP_IMPL(Check_NE, !=)  // Use CHECK(x == NULL) instead.
DEFINE_CHECK_OP_IMPL(Check_LE, <=)
DEFINE_CHECK_OP_IMPL(Check_LT, <)
DEFINE_CHECK_OP_IMPL(Check_GE, >=)
DEFINE_CHECK_OP_IMPL(Check_GT, >)
#undef DEFINE_CHECK_OP_IMPL

// CHECK dies with a fatal error if condition is not true.  It is *not*
// controlled by DCHECK_IS_ON(), so the check will be executed regardless of
// compilation mode.  Therefore, it is safe to do things like:
//    CHECK(fp->Write(x) == 4)
#define CHECK(condition)                                        \
  LOG_IF(FATAL, PICOLOG_PREDICT_BRANCH_NOT_TAKEN(!(condition))) \
      << "Check failed: " #condition " "

template <typename T>
T CheckNotNull(const char* file, int line, const char* names, T&& t) {
  if (t == nullptr) {
    LogMessageFatal(file, line, new std::string(names));
  }
  return std::forward<T>(t);
}

#define CHECK_OP_LOG(name, op, val1, val2, log)                                \
  while (::picolog::CheckOpString _result = ::picolog::Check##name##Impl(      \
             ::picolog::GetReferenceableValue(val1),                           \
             ::picolog::GetReferenceableValue(val2), #val1 " " #op " " #val2)) \
  log(__FILE__, __LINE__, _result).stream()

#if PICOLOG_STRIP_LOG <= 3
#define CHECK_OP(name, op, val1, val2) \
  CHECK_OP_LOG(name, op, val1, val2, ::picolog::LogMessageFatal)
#else
#define CHECK_OP(name, op, val1, val2) \
  CHECK_OP_LOG(name, op, val1, val2, ::picolog::NullStreamFatal)
#endif  // STRIP_LOG <= 3

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

#define CHECK_NOTNULL(val)                                                   \
  ::picolog::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", \
                          (val))

// TODO: this construction feels brittle
#define CHECK_OK(status) CHECK(status.ok()) << status

// Helper functions for string comparisons.
// To avoid bloat, the definitions are in logging.cc.
#define DECLARE_CHECK_STROP_IMPL(func, expected)                           \
  std::string* Check##func##expected##Impl(const char* s1, const char* s2, \
                                           const char* names);
DECLARE_CHECK_STROP_IMPL(strcmp, true)
DECLARE_CHECK_STROP_IMPL(strcmp, false)
DECLARE_CHECK_STROP_IMPL(strcasecmp, true)
DECLARE_CHECK_STROP_IMPL(strcasecmp, false)
#undef DECLARE_CHECK_STROP_IMPL

// Helper macro for string comparisons.
// Don't use this macro directly in your code, use CHECK_STREQ et al below.
#define CHECK_STROP(func, op, expected, s1, s2)                           \
  while (::picolog::CheckOpString _result =                               \
             ::picolog::Check##func##expected##Impl((s1), (s2),           \
                                                    #s1 " " #op " " #s2)) \
  LOG(FATAL) << *_result.str_

#define CHECK_STREQ(s1, s2) CHECK_STROP(strcmp, ==, true, s1, s2)
#define CHECK_STRNE(s1, s2) CHECK_STROP(strcmp, !=, false, s1, s2)
#define CHECK_STRCASEEQ(s1, s2) CHECK_STROP(strcasecmp, ==, true, s1, s2)
#define CHECK_STRCASENE(s1, s2) CHECK_STROP(strcasecmp, !=, false, s1, s2)

#define CHECK_INDEX(I, A) CHECK(I < (sizeof(A) / sizeof(A[0])))
#define CHECK_BOUND(B, A) CHECK(B <= (sizeof(A) / sizeof(A[0])))

#define VLOG(verboselevel) LOG_IF(INFO, VLOG_IS_ON(verboselevel))
#define VLOG_IF(verboselevel, condition) \
  LOG_IF(INFO, VLOG_IS_ON(verboselevel) && (condition))

}  // namespace picolog

// For whatever portions and snippets were copied from glog verbatim.
/*
 * Copyright (c) 2008, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * Modification, are permitted provided that the following conditions are
 * Met:
 *
 *     * Redistributions of source code must retain the above copyright
 * Notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * Copyright notice, this list of conditions and the following disclaimer
 * In the documentation and/or other materials provided with the
 * Distribution.
 *     * Neither the name of Google Inc. nor the names of its
 * Contributors may be used to endorse or promote products derived from
 * This software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#endif  // PICOLOG_LOGGING_H_