#ifndef TPLP_CONFIG_CONSTANTS_H_
#define TPLP_CONFIG_CONSTANTS_H_

#include <cstdint>

// Well-known values, expected not to change often if at all,
// and expected not to cause any dependency or coupling issues.

namespace tplp {

namespace constants {

static constexpr int32_t kLoadCellExpectedRangeAfterScaling = 1000;

// Used for pressure calculation.
// TODO: 300 is a guess, i need a multimeter!!
static constexpr int kTouchscreenResistanceOhmsX = 300;
static constexpr int kTouchscreenResistanceOhmsY = 300;

// Which feed direction (forward or reverse) is positive for the motor
// static constexpr int kMotorPolaritySrc = 1;
// static constexpr int kMotorPolarityDst = -1;

}  // namespace constants
}  // namespace tplp

#endif  // TPLP_CONFIG_CONSTANTS_H_