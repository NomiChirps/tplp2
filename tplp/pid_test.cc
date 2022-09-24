#include "tplp/pid.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using tplp::DerivativeEstimator;

TEST(DerivativeEstimator, BasicallyWorks) {
  DerivativeEstimator e(0);
  EXPECT_EQ(e.value(), 0);
  EXPECT_EQ(e.Update(100), 1100);
  EXPECT_EQ(e.Update(200), 400);
  EXPECT_EQ(e.Update(300), 600);
  EXPECT_EQ(e.Update(400), 600);
  EXPECT_EQ(e.Update(500), 600);
  EXPECT_EQ(e.Update(300), -2700);
  EXPECT_EQ(e.Update(200), 500);
  EXPECT_EQ(e.Update(100), -800);
  EXPECT_EQ(e.Update(100), 500);
  EXPECT_EQ(e.Update(100), -200);
  EXPECT_EQ(e.Update(100), 0);
  EXPECT_EQ(e.Update(100), 0);
  EXPECT_EQ(e.Update(100), 0);
}
