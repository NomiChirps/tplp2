#include "tplp/fs/crc7.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

TEST(crc7, SdSpecExample1) {
  uint8_t cmd0[] = {
      0b01000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  };
  ASSERT_EQ(crc7(0, cmd0, 5), 0b1001010);
}

TEST(crc7, SdSpecExample2) {
  uint8_t cmd17[] = {
      0b01010001, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  };
  ASSERT_EQ(crc7(0, cmd17, 5), 0b0101010);
}

TEST(crc7, SdSpecExample3) {
  uint8_t resp[] = {
      0b00010001, 0b00000000, 0b00000000, 0b00001001, 0b00000000,
  };
  ASSERT_EQ(crc7(0, resp, 5), 0b0110011);
}