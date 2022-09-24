#include "tplp/config/params.h"

#include <string>

#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/util.h"

namespace tplp {
namespace config {
namespace {

ABSL_CONST_INIT static int num_params = 0;
ABSL_CONST_INIT static ParameterBase* params[kMaxNumParams];
ABSL_CONST_INIT static util::Status deferred_init_error;

}  // namespace

ParameterBase::ParameterBase(const char* name, const char* help)
    : name_(name), help_(help) {
  if (num_params >= kMaxNumParams) {
    deferred_init_error = util::ResourceExhaustedError(
        "Too many parameters; increase kMaxNumParams");
    return;
  }
  if (IsInFlash(this)) {
    deferred_init_error = util::InternalError(
        "Parameter was placed in Flash memory instead of SRAM");
    return;
  }
  params[num_params++] = this;
}

ParameterBase::~ParameterBase() {}

AllParameters::iterator_t AllParameters::begin() const { return iterator_t(0); }
AllParameters::iterator_t AllParameters::end() const {
  return iterator_t(num_params);
}
ParameterBase* AllParameters::iterator_t::operator*() { return params[index_]; }

const util::Status& DeferredInitError() { return deferred_init_error; }

}  // namespace config
}  // namespace tplp
