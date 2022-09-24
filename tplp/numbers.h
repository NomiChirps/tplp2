#ifndef TPLP_NUMBERS_H_
#define TPLP_NUMBERS_H_

#include <cstdint>
#include <iomanip>
#include <ostream>

#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"

// Integer division, rounding up.
template <typename Int>
constexpr Int intdiv_ceil(Int x, Int y) {
  return (x / y) + (x % y != 0);
}

template <typename Int>
constexpr Int intpow(Int a, Int b) {
  Int result = 1;
  for (Int i = 0; i < b; ++i) {
    result *= a;
  }
  return result;
}

// struct rational32_t {
//   explicit rational32_t() : num(0), den(1) {}
//   explicit rational32_t(int32_t num) : num(num), den(1) {}
//   rational32_t(int32_t num, int32_t den) : num(num), den(den) {}

//   int32_t num;
//   int32_t den;

//   bool operator<(const rational32_t& rhs) {
//     return ((int64_t)num * rhs.den) < ((int64_t)den * rhs.num);
//   }

//   bool operator>(const rational32_t& rhs) {
//     return ((int64_t)num * rhs.den) > ((int64_t)den * rhs.num);
//   }

//   int32_t operator*(int32_t rhs) const { return ((int64_t)num * rhs) / den; }
// };

class fix32_base_t {};

template <int FractionalBits>
class fix32_t : public fix32_base_t {
 public:
  static constexpr int32_t kScale = 1 << FractionalBits;
  // Number of decimal digits in the fractional part
  static constexpr int kFracDigits = intdiv_ceil(FractionalBits * 1000, 3322);

  explicit constexpr fix32_t() : value_(0) {}
  explicit constexpr fix32_t(int32_t int_part, int32_t frac_part) {
    value_ = (int_part << FractionalBits) + frac_part;
  }

  static int32_t intmax() { return 0xffffffff >> kScale; }

  int32_t trunc() const { return value_ >> FractionalBits; }
  int32_t round() const {
    return trunc() + (fractional_part() > ((kScale - 1) / 2) ? 1 : 0);
  }
  // returns unsigned fractional part
  int32_t fractional_part() const { return value_ & (kScale - 1); }

  fix32_t operator*(int32_t rhs) const { return fix32_t(value_ * rhs); }
  fix32_t operator/(int32_t rhs) const { return fix32_t(value_ / rhs); }
  fix32_t operator+(int32_t rhs) const {
    return fix32_t(value_ + (rhs << FractionalBits));
  }
  fix32_t operator-(int32_t rhs) const {
    return fix32_t(value_ - (rhs << FractionalBits));
  }

  fix32_t operator-() const { return fix32_t(-value_); }
  fix32_t operator+(fix32_t rhs) const { return fix32_t(value_ + rhs.value_); }
  fix32_t operator-(fix32_t rhs) const { return fix32_t(value_ - rhs.value_); }
  fix32_t operator*(fix32_t rhs) const {
    return fix32_t(((int64_t)value_ * rhs.value_) >> FractionalBits);
  }
  // saturating and full precision. uses a 64-bit division.
  fix32_t operator/(fix32_t rhs) const {
    int64_t q = ((int64_t)value_ << FractionalBits) / rhs.value_;
    return fix32_t(q > 0xffffffff ? 0xffffffff : q);
  }

  fix32_t& operator+=(fix32_t rhs) {
    value_ += rhs.value_;
    return *this;
  }
  fix32_t& operator-=(fix32_t rhs) {
    value_ -= rhs.value_;
    return *this;
  }
  fix32_t& operator*=(fix32_t rhs) {
    value_ = ((int64_t)value_ * rhs.value_) >> FractionalBits;
    return *this;
  }
  fix32_t& operator/=(fix32_t rhs) {
    int64_t q = ((int64_t)value_ << FractionalBits) / rhs.value_;
    value_ = q > 0xffffffff ? 0xffffffff : q;
    return *this;
  }

  fix32_t& operator=(int32_t rhs) { return *this = fix32_t(rhs, 0); }

  std::string ToDecimalString() const;
  static bool FromDecimalString(std::string_view str, fix32_t* out);
  operator absl::AlphaNum() const { return ToDecimalString(); }

  bool operator<(fix32_t rhs) const { return value_ < rhs.value_; }
  bool operator<=(fix32_t rhs) const { return value_ <= rhs.value_; }
  bool operator>(fix32_t rhs) const { return value_ > rhs.value_; }
  bool operator>=(fix32_t rhs) const { return value_ >= rhs.value_; }
  bool operator==(fix32_t rhs) const { return value_ == rhs.value_; }
  bool operator!=(fix32_t rhs) const { return value_ != rhs.value_; }

 private:
  explicit fix32_t(int32_t value) : value_(value) {}

  int32_t value_;

  template <int F>
  friend fix32_t<F> operator/(int32_t lhs, fix32_t<F> rhs);
};

template <int FractionalBits>
fix32_t<FractionalBits> operator*(int32_t lhs, fix32_t<FractionalBits> rhs) {
  return rhs * lhs;
}

template <int FractionalBits>
fix32_t<FractionalBits> operator/(int32_t lhs, fix32_t<FractionalBits> rhs) {
  static_assert(FractionalBits <= 16);
  int64_t q = ((int64_t)lhs << (2 * FractionalBits)) / rhs.value_;
  return fix32_t<FractionalBits>(q > 0xffffffff ? 0xffffffff : q);
}

template <int FractionalBits>
std::string fix32_t<FractionalBits>::ToDecimalString() const {
  std::ostringstream s;
  if (trunc() == 0 && value_ < 0) {
    s << '-';
  }
  s << trunc() << "." << std::setfill('0')
    << std::setw(kFracDigits)
    // rescale frac part to decimal
    << ((int64_t)fractional_part() * intpow(10, kFracDigits)) / kScale;
  return s.str();
}

template <int FractionalBits>
bool fix32_t<FractionalBits>::FromDecimalString(std::string_view str,
                                                fix32_t<FractionalBits>* out) {
  auto decimal = str.find('.');
  if (decimal != std::string_view::npos) {
    auto int_part_str = str.substr(0, decimal);
    auto frac_part_str = str.substr(decimal + 1);
    int32_t int_part, frac_part;
    if (int_part_str.size() > 0) {
      if (!absl::SimpleAtoi(int_part_str, &int_part)) return false;
    } else {
      int_part = 0;
    }
    if (frac_part_str.size() > 0) {
      if (!absl::SimpleAtoi(frac_part_str, &frac_part)) return false;
    } else {
      frac_part = 0;
    }
    // round frac part if longer than kFracDigits
    auto frac_part_digits = frac_part_str.size();
    while (frac_part_digits > kFracDigits) {
      frac_part /= 10;
      frac_part_digits--;
    }
    // extend frac part with zeroes out to kFracDigits
    while (frac_part_digits < kFracDigits) {
      frac_part *= 10;
      frac_part_digits++;
    }
    // rescale frac part from decimal
    frac_part = ((int64_t)frac_part * kScale) / intpow(10, kFracDigits);
    // get negative sign if integer part was zero
    if (int_part == 0 &&
        str.substr(0, decimal).find('-') != std::string_view::npos) {
      frac_part *= -1;
    }
    out->value_ = (int_part << FractionalBits) + frac_part;
  } else {
    // no fractional part
    int32_t x;
    if (!absl::SimpleAtoi(str, &x)) return false;
    *out = x;
  }
  return true;
}

template <int FractionalBits>
std::ostream& operator<<(std::ostream& out, fix32_t<FractionalBits> rhs) {
  return out << rhs.ToDecimalString();
}

namespace util {
template <typename T>
int signum(T val) {
  return (T(0) < val) - (val < T(0));
}
}  // namespace util

#endif  // TPLP_NUMBERS_H_