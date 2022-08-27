#ifndef TPLP_CONFIG_PARAMS_H_
#define TPLP_CONFIG_PARAMS_H_

#include <cstdint>

// Well-known parameters, expected not to change often if at all,
// and expected not to cause any dependency or coupling issues.

namespace tplp {
namespace params {

static constexpr int32_t kLoadCellExpectedRangeAfterScaling = 1000;

}  // namespace params
}  // namespace tplp

#endif  // TPLP_CONFIG_PARAMS_H_