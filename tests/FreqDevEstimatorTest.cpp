#include "FreqDevEstimator.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class FreqDevEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(FreqDevEstimatorTest, construct)
{
    auto fde = mobilinkd::FreqDevEstimator<float>();
}

TEST_F(FreqDevEstimatorTest, fde_preamble)
{
    constexpr std::array<float, 24> input = {1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

    auto fde = mobilinkd::FreqDevEstimator<float>();
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x * 3);});
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x * 3);});
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x * 3);});

    fde.update();

    EXPECT_NEAR(fde.deviation(), 1, .1);
    EXPECT_NEAR(fde.error(), 0, .1);
}

TEST_F(FreqDevEstimatorTest, fde_mixed)
{
    constexpr std::array<float, 16> input = {1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1};

    auto fde = mobilinkd::FreqDevEstimator<float>();
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x * 3);});
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x);});
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x * 3);});
    std::for_each(input.begin(), input.end(), [&fde](float x){fde.sample(x);});

    fde.update();

    EXPECT_NEAR(fde.deviation(), 1, .1);
    EXPECT_NEAR(fde.error(), 0, .1);
}
