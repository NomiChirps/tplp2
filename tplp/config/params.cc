#include "tplp/config/params.h"

#include <string>

#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"

namespace tplp {
namespace config {
namespace {

static constexpr int kMaxNumParams = 32;
// Directory on the filesystem where parameter values shall be stored.
static const char* const kConfigDirectory = "params";

ABSL_CONST_INIT static int num_params = 0;
ABSL_CONST_INIT static ParameterBase* params[kMaxNumParams];
ABSL_CONST_INIT static util::Status deferred_init_error;

// Generates an 8.3 file path inspired by the given parameter name.
static std::string GenerateFilePath(std::string_view param_name) {
  if (param_name.size() <= 8) {
    return absl::StrCat(kConfigDirectory, "/", param_name, ".flg");
  } else {
    return absl::StrCat(kConfigDirectory, "/", param_name.substr(0, 4), "~",
                        param_name.substr(param_name.size() - 4, 3), ".flg");
  }
}

}  // namespace

ParameterBase::ParameterBase(const char* name, const char* help)
    : name_(name), help_(help), file_path_(GenerateFilePath(name)) {
  if (num_params >= kMaxNumParams) {
    deferred_init_error = util::ResourceExhaustedError(
        "Too many parameters; increase kMaxNumParams");
    return;
  }
  params[num_params++] = this;
}

ParameterBase::~ParameterBase() {}

template <typename T>
Parameter<T>::Parameter(const char* name, const T& default_value,
                        const char* help)
    : ParameterBase(name, help), value_(default_value) {}

AllParameters::iterator_t AllParameters::begin() const { return iterator_t(0); }
AllParameters::iterator_t AllParameters::end() const {
  return iterator_t(num_params);
}
ParameterBase* AllParameters::iterator_t::operator*() { return params[index_]; }

// TODO: extend to any numeric type
template <>
util::Status Parameter<uint32_t>::Parse(std::string_view str) {
  if (absl::SimpleAtoi(str, &value_)) {
    return util::OkStatus();
  } else {
    return util::InvalidArgumentError("parse error");
  }
}

// TODO: extend to any numeric type
template <>
util::StatusOr<size_t> Parameter<uint32_t>::Serialize(char* buf,
                                                      size_t n) const {
  auto str = absl::StrCat(value_);
  if (str.size() > n) {
    return util::ResourceExhaustedError("serialized value too large");
  }
  return str.copy(buf, n);
}

// TODO: extend to any numeric type
template <>
std::string Parameter<uint32_t>::DebugString() const {
  return absl::StrCat(value_);
}

const util::Status& DeferredInitError() { return deferred_init_error; }

}  // namespace config
}  // namespace tplp
