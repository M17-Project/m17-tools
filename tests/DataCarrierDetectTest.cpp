#include "DataCarrierDetect.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class DataCarrierDetectTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(DataCarrierDetectTest, construct)
{
    auto dcd = mobilinkd::DataCarrierDetect<float, 48000, 1000>(2000, 4000, 1.0, 5.0);
}

TEST_F(DataCarrierDetectTest, dcd)
{
    constexpr std::array<float, 24> input = {1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

    auto dcd = mobilinkd::DataCarrierDetect<float, 48000, 1000>(2000,3000,1.0,5.0);
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});

    dcd.update();

    EXPECT_TRUE(dcd.dcd());
}

TEST_F(DataCarrierDetectTest, dcd_off)
{
    constexpr std::array<float, 16> input = {1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1};

    auto dcd = mobilinkd::DataCarrierDetect<float, 48000, 1000>(2000,3000, 0.1, 1.0);
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});
    std::for_each(input.begin(), input.end(), [&dcd](float x){dcd(x);});

    dcd.update();

    EXPECT_FALSE(dcd.dcd());
}
