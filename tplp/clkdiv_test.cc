#include "tplp/clkdiv.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using tplp::ClockDivider;
using tplp::ComputeClockDivider;

TEST(ComputeClockDivider, TooSlow) {
  int sys_hz = 125'000'000;
  ClockDivider clkdiv;
  EXPECT_FALSE(ComputeClockDivider(sys_hz, sys_hz / 65535 - 1, &clkdiv))
      << clkdiv;
  EXPECT_TRUE(ComputeClockDivider(sys_hz, sys_hz / 65535 + 1, &clkdiv))
      << clkdiv;
}

TEST(ComputeClockDivider, TooFast) {
  int sys_hz = 125'000'000;
  ClockDivider clkdiv;
  EXPECT_TRUE(ComputeClockDivider(sys_hz, sys_hz + 0, &clkdiv)) << clkdiv;
  EXPECT_FALSE(ComputeClockDivider(sys_hz, sys_hz + 1, &clkdiv)) << clkdiv;
}

class ComputeClockDividerTest : public testing::TestWithParam<int> {
 public:
  void SetUp() override {
    sys_hz_ = 125'000'000;
    // We can get QUITE good approximations, it turns out!
    tolerance_ = std::max(GetParam() * 0.000005, 1.);
  }

 protected:
  int sys_hz_;
  int tolerance_;
};

TEST_P(ComputeClockDividerTest, WithinTolerance) {
  ClockDivider clkdiv;
  ASSERT_TRUE(ComputeClockDivider(sys_hz_, GetParam(), &clkdiv));
  EXPECT_NEAR((float)sys_hz_ * clkdiv.num / clkdiv.den, GetParam(), tolerance_)
      << ">>> " << clkdiv.num << " / " << clkdiv.den
      << " == " << ((float)clkdiv.num / (float)clkdiv.den);
}

INSTANTIATE_TEST_SUITE_P(Evens, ComputeClockDividerTest,
                         testing::Range(2'000, 125'000'000, 10'000));

INSTANTIATE_TEST_SUITE_P(Primes, ComputeClockDividerTest,
                         testing::Range(2'000, 125'000'000, 7919));