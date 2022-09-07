#include "tplp/clkdiv.h"

#include <cstdint>
#include <limits>
#include <memory>

#include "picolog/picolog.h"

namespace tplp {

bool CalculateClockDivider(int sys_hz, int target_hz,
                           ClockDivider* out_clkdiv) {
  constexpr int kMaxCf = 8;
  if (target_hz < 0) {
    LOG(ERROR) << "CalculateClockDivider: speed must be positive";
    return false;
  }
  if (target_hz >= sys_hz) {
    LOG(ERROR) << "Cannot divide faster than the system clock (" << sys_hz
               << "). Requested "
                  "target_hz = "
               << target_hz;
    return false;
  }
  // y * sys_hz / x = target_hz
  // y / x = target_hz / sys_hz
  // NB: we'll be calculating stuff with x and y swapped
  //     so that the fraction is greater than 1.
  const int y = target_hz;
  const int x = sys_hz;
  // we need the closest rational approximation to x/y
  // such that both the numerator and the denominator
  // fit into a uint16_t. strap in we're gonna do math.
  // continued fraction expansion of x/y:
  int cf[kMaxCf];
  int cf_len = 0;
  int num = x;
  int den = y;
  for (; cf_len < kMaxCf;) {
    int whole = num / den;
    int remainder = num % den;
    cf[cf_len++] = whole;
    VLOG(1) << "cf[" << cf_len - 1 << "] = " << whole;
    if (!remainder) break;
    num = den;
    den = remainder;
  }
  CHECK_GT(cf_len, 0);
  // calculate rational approximations from right to left
  int nds[cf_len][2];
  nds[cf_len - 1][0] = cf[0];
  nds[cf_len - 1][1] = 1;
  if (nds[cf_len - 1][0] > std::numeric_limits<uint16_t>::max() ||
      nds[cf_len - 1][1] > std::numeric_limits<uint16_t>::max()) {
    // the necessary clock divider is too large and can't be represented.
    LOG(WARNING) << "Cannot divide lower than sys_clk/65535==" << sys_hz / 65535
                 << ". Requested target_hz = " << target_hz;
    return false;
  }
  VLOG(1) << x << " / " << y << " ~ " << nds[cf_len - 1][0] << " / "
          << nds[cf_len - 1][1];
  bool found = false;
  int approx_num;
  int approx_den;
  for (int i = cf_len - 2; i >= 0; --i) {
    nds[i][0] = cf[i] * nds[i + 1][0] + nds[i + 1][1];
    nds[i][1] = nds[i + 1][0];
    VLOG(1) << x << " / " << y << " ~ " << nds[i][0] << " / " << nds[i][1];
    // is this one too big?
    if (nds[i][0] > std::numeric_limits<uint16_t>::max() ||
        nds[i][1] > std::numeric_limits<uint16_t>::max()) {
      // use the previous one
      approx_num = nds[i + 1][0];
      approx_den = nds[i + 1][1];
      found = true;
      break;
    }
  }
  if (!found) {
    // cool, all of the approximations will fit;
    // use the closest one
    approx_num = nds[0][0];
    approx_den = nds[0][1];
  }
  CHECK_GT(approx_num, 0);
  CHECK_GT(approx_den, 0);

  // See comment at declaration of x and y.
  std::swap(approx_num, approx_den);
  VLOG(1) << "CalculateClockDivider(" << target_hz << ") == " << approx_num
          << " / " << approx_den << "; error ~ "
          << (approx_num * (sys_hz / approx_den)) - target_hz << "Hz";
  out_clkdiv->num = approx_num;
  out_clkdiv->den = approx_den;
  return true;
}

}  // namespace tplp