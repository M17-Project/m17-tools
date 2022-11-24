#include "M17Framer.h"

#include <gtest/gtest.h>

#include <cstdint>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/ConvolutionTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class M17FramerTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(M17FramerTest, construct)
{
    mobilinkd::M17Framer framer;
}
