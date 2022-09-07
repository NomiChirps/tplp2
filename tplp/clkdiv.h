#ifndef TPLP_CLKDIV_H_
#define TPLP_CLKDIV_H_

#include <cstdint>

namespace tplp {

struct ClockDivider {
  uint16_t num;
  uint16_t den;
};

// Calculates the clock divider value which most closely approximates the
// given target frequency. Both the system frequency `sys_hz` and the target
// frequency `target_hz` must be positive. This is a complicated calculation and
// could take up to a few hundred cycles.
//
// Returns true if successful, false if the given frequency is out of range
// and not achievable.
bool CalculateClockDivider(int sys_hz, int target_hz, ClockDivider* out_clkdiv);

}  // namespace tplp

#endif  // TPLP_MOTOR_CLKDIV_H_