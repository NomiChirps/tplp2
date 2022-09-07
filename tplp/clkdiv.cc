#include "tplp/clkdiv.h"

#include <cstdint>
#include <limits>
#include <memory>

#include "picolog/picolog.h"

namespace tplp {
namespace {
struct Rational64 {
  int64_t num;
  int64_t den;

  bool operator<(const Rational64& rhs) {
    return (num * rhs.den) < (den * rhs.num);
  }
  bool operator>(const Rational64& rhs) {
    return (num * rhs.den) > (den * rhs.num);
  }
};

Rational64 mediant(Rational64 a, Rational64 b) {
  return {a.num + b.num, a.den + b.den};
}

// returns true if |f-x| < |b-x|, i.e.,
// if f is closer to x than b is.
bool IsCloser(Rational64 x, Rational64 f, Rational64 b) {
  if (f < x) {
    if (b < x) {
      return b < f;
    } else {
      return 2 * x.num * b.den * f.den <
             b.num * x.den * f.den + f.num * x.den * b.den;
    }
  } else {
    if (b < x) {
      return 2 * x.num * b.den * f.den >
             b.num * x.den * f.den + f.num * x.den * b.den;
    } else {
      return b > f;
    }
  }
}

// https://math.stackexchange.com/questions/2555205/best-rational-approximation-with-numerator-denominator-less-than-255
Rational64 Approximate(Rational64 x, int max_den) {
  Rational64 l, r, f, b;
  // left
  l = {1, 0};
  // right
  r = {0, 1};
  // current fraction
  f = {1, 1};
  // best fraction
  b = {1, 1};

  for (;;) {
    if (x < f) {
      l = f;
      f = mediant(r, f);
    } else if (x > f) {
      r = f;
      f = mediant(l, f);
    } else {
      return f;
    }
    if (f.den > max_den) {
      return b;
    }
    if (IsCloser(x, f, b)) {
      b = f;
    }
  }
}

}  // namespace

bool ComputeClockDivider(int sys_hz, int target_hz, ClockDivider* out_clkdiv) {
  if (target_hz < 0) {
    VLOG(1) << "ComputeClockDivider: target frequency must be positive";
    return false;
  }
  if (target_hz > sys_hz) {
    VLOG(1) << "Cannot divide to faster than the system clock (" << sys_hz
            << "). Requested "
               "target_hz = "
            << target_hz;
    return false;
  }
  if (target_hz < sys_hz / 65535) {
    VLOG(1) << "Cannot divide to slower than sys_hz/65535 (" << sys_hz / 65535
            << ". Requested target_hz = " << target_hz;
    return false;
  }
  Rational64 clkdiv = Approximate({target_hz, sys_hz}, 65535);
  CHECK_GT(clkdiv.num, 0);
  CHECK_LE(clkdiv.den, 65535);

  out_clkdiv->num = clkdiv.num;
  out_clkdiv->den = clkdiv.den;
  return true;
}

std::ostream& operator<<(std::ostream& out, const ClockDivider& self) {
  return out << "(" << self.num << "/" << self.den << ")";
}

}  // namespace tplp