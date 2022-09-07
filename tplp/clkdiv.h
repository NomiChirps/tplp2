#ifndef TPLP_CLKDIV_H_
#define TPLP_CLKDIV_H_

#include <cstdint>
#include <ostream>

namespace tplp {

struct ClockDivider {
  uint16_t num;
  uint16_t den;

  friend std::ostream& operator<<(std::ostream&, const ClockDivider&);
};

// Computes the unique 16-bit clock divider value which most closely
// approximates the given target frequency. Both the system frequency `sys_hz`
// and the target frequency `target_hz` must be positive. This is a complicated
// calculation and could take a few hundred cycles (or longer. I haven't
// characterized it.)
//
// Returns true if successful, false if the given frequency is out of range
// and not achievable.
//
// Error is expected to be extremely small (<0.001%) for reasonable values of
// sys_hz (~MHz) and target_hz (~kHz-MHz).
bool ComputeClockDivider(int sys_hz, int target_hz, ClockDivider* out_clkdiv);

}  // namespace tplp

#endif  // TPLP_MOTOR_CLKDIV_H_