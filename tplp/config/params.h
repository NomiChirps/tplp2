#ifndef TPLP_CONFIG_PARAMS_H_
#define TPLP_CONFIG_PARAMS_H_

#include <cstdint>

// Well-known parameters, expected not to change often if at all,
// and expected not to cause any dependency or coupling issues.

namespace tplp {
namespace params {

static constexpr int32_t kLoadCellExpectedRangeAfterScaling = 1000;

// Used for pressure calculation.
// TODO: 300 is a guess, i need a multimeter!!
static constexpr int kTouchscreenResistanceOhmsX = 300;
static constexpr int kTouchscreenResistanceOhmsY = 300;

}  // namespace params
}  // namespace tplp

#endif  // TPLP_CONFIG_PARAMS_H_