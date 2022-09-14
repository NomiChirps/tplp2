#include "tplp/config/flags.h"

#include <string>

#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/fs/fs.h"

namespace tplp {
namespace config {
namespace {

static constexpr int kMaxNumFlags = 32;
static constexpr size_t kMaxSerializedValueLen = 512;
// Directory on the filesystem where flag values shall be stored.
static const char* const kConfigDirectory = "flags";

ABSL_CONST_INIT static int num_flags__ = 0;
ABSL_CONST_INIT static FlagBase* flags__[kMaxNumFlags];
ABSL_CONST_INIT static util::Status deferred_init_error__;

// Generates an 8.3 file path inspired by the given flag name.
static std::string GenerateFilePath(std::string_view flag_name) {
  if (flag_name.size() <= 8) {
    return absl::StrCat(kConfigDirectory, "/", flag_name, ".flg");
  } else {
    return absl::StrCat(kConfigDirectory, "/", flag_name.substr(0, 4), "~",
                        flag_name.substr(flag_name.size() - 4, 3), ".flg");
  }
}

}  // namespace

FlagBase::FlagBase(const char* name, const char* help)
    : name_(name), help_(help), file_path_(GenerateFilePath(name)) {
  if (num_flags__ >= kMaxNumFlags) {
    deferred_init_error__ =
        util::ResourceExhaustedError("Too many flags; increase kMaxNumFlags");
    return;
  }
  flags__[num_flags__++] = this;
}

FlagBase::~FlagBase() {}

template <typename T>
Flag<T>::Flag(const char* name, const T& default_value, const char* help)
    : FlagBase(name, help), value_(default_value) {}

util::Status FlagBase::Save() const {
  char buf[kMaxSerializedValueLen];
  auto bytes_count = Serialize(buf, kMaxSerializedValueLen);
  if (!bytes_count.ok()) {
    LOG(ERROR) << "Error serializing flag value " << name() << ": "
               << bytes_count.status();
    return util::InternalError("serialization error");
  }
  util::Status status = fs::SetContents(file_path(), buf, *bytes_count);
  if (!status.ok()) {
    LOG(ERROR) << "Error writing " << file_path() << ": " << status;
    return status;
  }
  return util::OkStatus();
}

util::Status FlagBase::Load() {
  char buf[kMaxSerializedValueLen];
  auto bytes_read = fs::GetContents(file_path(), buf, kMaxSerializedValueLen);
  if (!bytes_read.ok()) {
    LOG(ERROR) << "Error reading " << file_path() << ": "
               << bytes_read.status();
    return std::move(bytes_read).status();
  }
  auto status = Parse(std::string_view(buf, *bytes_read));
  if (!status.ok()) {
    LOG(ERROR) << "Error parsing flag value " << name() << ": " << status;
    return status;
  }
  return util::OkStatus();
}
AllFlags::iterator_t AllFlags::begin() const { return iterator_t(0); }
AllFlags::iterator_t AllFlags::end() const { return iterator_t(num_flags__); }
FlagBase* AllFlags::iterator_t::operator*() { return flags__[index_]; }

util::Status LoadAllFlags() {
  RETURN_IF_ERROR(deferred_init_error__);
  for (FlagBase* flag : AllFlags()) {
    RETURN_IF_ERROR(flag->Load());
  }
  return util::OkStatus();
}

util::Status SaveAllFlags() {
  RETURN_IF_ERROR(deferred_init_error__);
  for (FlagBase* flag : AllFlags()) {
    RETURN_IF_ERROR(flag->Save());
  }
  return util::OkStatus();
}

// TODO: extend to any numeric type
template <>
util::Status Flag<uint32_t>::Parse(std::string_view str) {
  if (absl::SimpleAtoi(str, &value_)) {
    return util::OkStatus();
  } else {
    return util::InvalidArgumentError("parse error");
  }
}

// TODO: extend to any numeric type
template <>
util::StatusOr<size_t> Flag<uint32_t>::Serialize(char* buf, size_t n) const {
  return absl::StrCat(value_).copy(buf, n);
}

}  // namespace config
}  // namespace tplp
