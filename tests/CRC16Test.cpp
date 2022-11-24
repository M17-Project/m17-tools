#include "CRC16.h"

#include <gtest/gtest.h>

#include <cstdint>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class CRC16Test : public ::testing::Test {
 protected:
  void SetUp() override { crc.reset(); }

  // void TearDown() override {}

  mobilinkd::CRC16<0x5935, 0xFFFF> crc;
};

TEST_F(CRC16Test, init)
{
    auto checksum = crc.get();
    EXPECT_EQ(checksum, 0xFFFF);  
}

TEST_F(CRC16Test, A)
{
    crc('A');
    auto checksum = crc.get();
    EXPECT_EQ(checksum, 0x206E);  
}

TEST_F(CRC16Test, numbers)
{
    crc('1');
    crc('2');
    crc('3');
    crc('4');
    crc('5');
    crc('6');
    crc('7');
    crc('8');
    crc('9');
    auto checksum = crc.get();
    EXPECT_EQ(checksum, 0x772B);  
}

TEST_F(CRC16Test, all)
{
    for (uint16_t i = 0; i != 256; ++i) crc(uint8_t(i));
    auto checksum = crc.get();
    EXPECT_EQ(checksum, 0x1C31);  
}

