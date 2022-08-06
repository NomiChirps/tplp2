#include "picolog/picolog.h"

#include <cxxabi.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/task.h"
#include "pico/bootrom.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "picolog/backtrace.h"

namespace picolog {
namespace {

const char* const LogSeverityNames[NUM_SEVERITIES] = {"INFO", "WARNING",
                                                      "ERROR", "FATAL"};

// Get the part of filepath after the last path separator.
const char* const_basename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

// Returns current thread/task name
const char* GetTID() {
  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  if (self) {
    return pcTaskGetName(self);
  } else {
    return "";
  }
}

}  // namespace

// An arbitrary limit on the length of a single log message. This
// is so that streaming can be done more efficiently.
// TODO: these should be compile-time configurable somehow?
const size_t LogMessage::kMaxLogMessageLen = 512;
const size_t kMaxBacktraceLength = 8;

struct LogMessage::LogMessageData {
  LogMessageData() : stream_(message_text_, LogMessage::kMaxLogMessageLen) {}

  // Buffer space; contains complete message text.
  char message_text_[LogMessage::kMaxLogMessageLen + 1];
  LogStream stream_;  // TODO: move to LogMessage! it's huge!

  LogMessageTime logmsgtime_;
  LogSeverity severity_;     // What level is this LogMessage logged at?
  const char* basename_;     // basename of file that called LOG
  const char* fullname_;     // fullname of file that called LOG
  const char* task_name_;    // name of the task that called LOG
  int line_;                 // line number where logging call is.
  size_t num_chars_to_log_;  // # of chars of msg to send to log
  bool has_been_flushed_;    // TODO: move to LogMessage
  bool first_fatal_;         // true => this was first fatal msg
  backtrace_frame_t backtrace_[kMaxBacktraceLength];
  int backtrace_length_;

 private:
  LogMessageData(const LogMessageData&) = delete;
  void operator=(const LogMessageData&) = delete;
};

// Globals
namespace {
using QueueContent = LogMessage::LogMessageData;
static constexpr int kQueueLength = 8;
static uint8_t queue_storage[kQueueLength * sizeof(QueueContent)];
static StaticQueue_t queue_structure;
static QueueHandle_t queue;

static_assert(sizeof(QueueContent) * kQueueLength * 4 < configTOTAL_HEAP_SIZE,
              "Log queue would take up more than 1/4 of the FreeRTOS heap; "
              "are you sure that's reasonable?");
}  // namespace

// As a belt-and-suspenders move, ask GCC to run this before main().
// Just in case the user forgot.
[[gnu::constructor]] void InitLogging() {
  if (!queue) {
    queue = xQueueCreateStatic(kQueueLength, sizeof(QueueContent),
                               queue_storage, &queue_structure);
    if (!queue) {
      panic("Failed to create picolog event queue");
    }

    // disable newlib's stdout buffering
    setvbuf(stdout, nullptr, _IONBF, 0);
  }
}

LogMessage::LogMessage(const char* file, int line, const CheckOpString& result)
    : data_(NULL) {
  Init(file, line, PICOLOG_FATAL);
  stream() << "Check failed: " << (*result.str_) << " ";
}

LogMessage::LogMessage(const char* file, int line) : data_(NULL) {
  Init(file, line, PICOLOG_INFO);
}

LogMessage::LogMessage(const char* file, int line, LogSeverity severity)
    : data_(NULL) {
  Init(file, line, severity);
}

void LogMessage::Init(const char* file, int line, LogSeverity severity) {
  // TODO: consider using thread-local storage
  // (this won't play nice with multicore without extra effort)
  absolute_time_t timestamp_now = get_absolute_time();
  data_ = new LogMessageData();
  data_->logmsgtime_ = LogMessageTime(to_us_since_boot(timestamp_now));
  data_->severity_ = severity;
  data_->line_ = line;
  data_->num_chars_to_log_ = 0;
  data_->basename_ = const_basename(file);
  data_->fullname_ = file;
  data_->task_name_ = GetTID();
  data_->has_been_flushed_ = false;
  data_->backtrace_length_ = 0;

  // clang-format off
  stream() << std::setfill('0')
           << LogSeverityNames[severity][0]
           << std::setw(2) << data_->logmsgtime_.hour() << ':'
           << std::setw(2) << data_->logmsgtime_.min() << ':'
           << std::setw(2) << data_->logmsgtime_.sec() << '.'
           << std::setw(6) << data_->logmsgtime_.usec()
           << ' '
           << std::setfill(' ') << std::setw(5) << data_->task_name_
           << ' '
           << data_->basename_ << ':' << data_->line_ << "] ";
  // clang-format on
  // fill character remains ' ', which is the default.
}

std::ostream& LogMessage::stream() {
  // TODO: check log suppression settings and return a null stream instead
  return data_->stream_;
}

LogMessage::~LogMessage() {
  Flush();
  delete data_;
}

void LogMessage::Flush() {
  if (data_->has_been_flushed_) {
    return;
  }
  // TODO: short-circuit here when logging is disabled/filtered at runtime.

  if (data_->severity_ == PICOLOG_FATAL) {
    // TODO: it would be cool if we could automatically break into the debugger
    // from here. unfortunately this doesn't seem to work.
    /*
    #if
        !defined(PICOLOG_BKPT_ON_FATAL) || PICOLOG_BKPT_ON_FATAL
            // See DHCSR description in ARMv6-M Architecture Reference Manual
            volatile const uint32_t* dhcsr = (uint32_t*)0xE000EDF0;
        if (*dhcsr & 0x1) {
          // stop here if we're running under a debugger
          asm volatile("bkpt #0");
        }
    #endif
    */
    data_->backtrace_length_ =
        TakeBacktrace(data_->backtrace_, kMaxBacktraceLength);
  }

  data_->num_chars_to_log_ = data_->stream_.pcount();

  // Do we need to add a \n to the end of this message?
  bool append_newline =
      (data_->message_text_[data_->num_chars_to_log_ - 1] != '\n');
  char original_final_char = '\0';

  // If we do need to add a \n, we'll do it by violating the memory of the
  // ostrstream buffer.  This is quick, and we'll make sure to undo our
  // modification before anything else is done with the ostrstream.  It
  // would be preferable not to do things this way, but it seems to be
  // the best way to deal with this.
  if (append_newline) {
    original_final_char = data_->message_text_[data_->num_chars_to_log_];
    data_->message_text_[data_->num_chars_to_log_++] = '\n';
  }
  data_->message_text_[data_->num_chars_to_log_] = '\0';

  if (xQueueSendToBack(queue, data_, 0) != pdTRUE) {
    // TODO: maybe keep a count of dropped messages somewhere.
    // thread-local counters that are summed by the log task ?
  }

  if (append_newline) {
    // Fix the ostrstream back how it was before we screwed with it.
    // It's 99.44% certain that we don't need to worry about doing this.
    data_->message_text_[data_->num_chars_to_log_ - 1] = original_final_char;
  }

  // Note that this message is now safely logged.  If we're asked to flush
  // again, as a result of destruction, say, we'll do nothing on future calls.
  data_->has_been_flushed_ = true;
}

void LogMessage::Fail() {
  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  if (self) {
    vTaskSuspend(self);
    panic("Cannot resume task after a fatal error");
  }
  panic("Fatal error");
}

LogMessageFatal::LogMessageFatal(const char* file, int line)
    : LogMessage(file, line, PICOLOG_FATAL) {}

LogMessageFatal::LogMessageFatal(const char* file, int line,
                                 const CheckOpString& result)
    : LogMessage(file, line, result) {}

LogMessageFatal::~LogMessageFatal() {
  Flush();
  LogMessage::Fail();
}

static constexpr std::time_t kMicrosecondsPerSecond = 1'000'000;
static constexpr std::time_t kMicrosecondsPerMinute =
    60 * kMicrosecondsPerSecond;
static constexpr std::time_t kMicrosecondsPerHour = 60 * kMicrosecondsPerMinute;

int LogMessageTime::hour() const {
  return usecs_since_boot_ / kMicrosecondsPerHour;
}
int LogMessageTime::min() const {
  return (usecs_since_boot_ % kMicrosecondsPerHour) / kMicrosecondsPerMinute;
}
int LogMessageTime::sec() const {
  return (usecs_since_boot_ % kMicrosecondsPerMinute) / kMicrosecondsPerSecond;
}
int LogMessageTime::usec() const {
  return usecs_since_boot_ % kMicrosecondsPerSecond;
}

static void PrintStackTrace(const backtrace_frame_t* frames, int count) {
  // __cxa_demangle claims to automatically realloc() the buffer if it's not
  // large enough. wild.
  char* demangled = static_cast<char*>(malloc(40));
  size_t demangled_len = 40;
  int status;
  for (int i = 0; i < count; ++i) {
    demangled =
        abi::__cxa_demangle(frames[i].name, demangled, &demangled_len, &status);
    if (status == 0) {
      printf("    @ %p %.*s\n", frames[i].ip, demangled_len, demangled);
    } else {
      // demangling failed
      printf("    @ %p %s\n", frames[i].ip, frames[i].name);
    }
  }
  if (count == kMaxBacktraceLength) {
    printf("  <backtrace limit reached>\n");
  }
  free(demangled);
}

void BackgroundTask(void*) {
  static std::byte recv_buf[sizeof(LogMessage::LogMessageData)];

  for (;;) {
    while (!xQueueReceive(queue, recv_buf, portMAX_DELAY)) {
      // TODO: it would be nice to count the number of dropped log messages if
      // the queue fills up. best way would be an atomic increment from tasks
      // (if we can?), then here change portMAX_DELAY to some small-ish value.
      // if the delay runs out then we probably have time to print a message
      // about how many other messages were dropped.
      // Or maybe it would make more sense to print that the moment we see the
      // queue is empty, instead? maybe inefficient to check the queue's status
      // twice every loop.
    }
    LogMessage::LogMessageData* data =
        reinterpret_cast<LogMessage::LogMessageData*>(recv_buf);
    // We disabled stdout's buffering earlier, so this should go straight to the
    // pico-sdk _write() function, which always flushes.
    fwrite(data->message_text_, data->num_chars_to_log_, 1, stdout);

    if (data->severity_ == PICOLOG_FATAL) {
      // TODO: suspend others? what about the printf/malloc mutex?
      // vTaskSuspendAll();
      printf("\n*** Aborted at %lluus since boot ***\n",
             to_us_since_boot(get_absolute_time()));
      PrintStackTrace(data->backtrace_, data->backtrace_length_);
      fflush(stdout);  // just in case
#if PICOLOG_RESET_TO_BOOTLOADER_ON_FATAL
      // TODO maybe there's a better way to configure this
      printf("\n*** Resetting to Pico bootloader!\n");
      reset_usb_boot(0, 0);
#else
      panic("Aborted by fatal error handler");
#endif
    }
  }
}

// Helper functions for string comparisons.
#define DEFINE_CHECK_STROP_IMPL(name, func, expected)                         \
  std::string* Check##func##expected##Impl(const char* s1, const char* s2,    \
                                           const char* names) {               \
    bool equal = s1 == s2 || (s1 && s2 && !func(s1, s2));                     \
    if (equal == expected)                                                    \
      return NULL;                                                            \
    else {                                                                    \
      std::ostringstream ss;                                                  \
      if (!s1) s1 = "";                                                       \
      if (!s2) s2 = "";                                                       \
      ss << #name " failed: " << names << " (" << s1 << " vs. " << s2 << ")"; \
      return new std::string(ss.str());                                       \
    }                                                                         \
  }
DEFINE_CHECK_STROP_IMPL(CHECK_STREQ, strcmp, true)
DEFINE_CHECK_STROP_IMPL(CHECK_STRNE, strcmp, false)
// strcasecmp doesn't seem to be available under arm-none-eabi-gcc
// DEFINE_CHECK_STROP_IMPL(CHECK_STRCASEEQ, strcasecmp, true)
// DEFINE_CHECK_STROP_IMPL(CHECK_STRCASENE, strcasecmp, false)
#undef DEFINE_CHECK_STROP_IMPL

CheckOpMessageBuilder::CheckOpMessageBuilder(const char* exprtext)
    : stream_(new std::ostringstream) {
  *stream_ << exprtext << " (";
}

CheckOpMessageBuilder::~CheckOpMessageBuilder() { delete stream_; }

std::ostream* CheckOpMessageBuilder::ForVar2() {
  *stream_ << " vs. ";
  return stream_;
}

std::string* CheckOpMessageBuilder::NewString() {
  *stream_ << ")";
  return new std::string(stream_->str());
}

template <>
void MakeCheckOpValueString(std::ostream* os, const char& v) {
  if (v >= 32 && v <= 126) {
    (*os) << "'" << v << "'";
  } else {
    (*os) << "char value " << static_cast<short>(v);
  }
}

template <>
void MakeCheckOpValueString(std::ostream* os, const signed char& v) {
  if (v >= 32 && v <= 126) {
    (*os) << "'" << v << "'";
  } else {
    (*os) << "signed char value " << static_cast<short>(v);
  }
}

template <>
void MakeCheckOpValueString(std::ostream* os, const unsigned char& v) {
  if (v >= 32 && v <= 126) {
    (*os) << "'" << v << "'";
  } else {
    (*os) << "unsigned char value " << static_cast<unsigned short>(v);
  }
}

template <>
void MakeCheckOpValueString(std::ostream* os, const std::nullptr_t& /*v*/) {
  (*os) << "nullptr";
}

}  // namespace picolog
