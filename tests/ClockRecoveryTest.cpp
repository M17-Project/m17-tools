#include "ClockRecovery.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/SlidingDFTTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class ClockRecoveryTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(ClockRecoveryTest, construct)
{
    auto cr = mobilinkd::ClockRecovery<float, 48000, 4800>();
}

TEST_F(ClockRecoveryTest, recover_preamble)
{
    // 2400Hz sine wave -- same as M17 preamble.
    constexpr std::array<double, 20> input = {
        0.0, 0.30901699437494745, 0.5877852522924732, 0.8090169943749475,
        0.9510565162951536, 1.0, 0.9510565162951536, 0.8090169943749475,
        0.5877852522924728, 0.3090169943749471, 0.0, -0.30901699437494773,
        -0.587785252292473, -0.8090169943749479, -0.9510565162951535, -1.0,
        -0.9510565162951533, -0.8090169943749476, -0.5877852522924726, -0.3090169943749468
    };
    
    {
        auto cr = mobilinkd::ClockRecovery<float, 48000, 4800>();
        for (size_t i = 0; i != 48; ++i) // Half of M17 preamble.
        {
            for (size_t j = 0; j != input.size(); ++j)
            {
                cr(input[j]);
            }
        }
        cr.update();
        EXPECT_EQ(cr.clock_estimate(), 1.0);
        EXPECT_EQ(int(cr.sample_index()), 5);
        // Verify we get actual best sample points.
        EXPECT_EQ(input[cr.sample_index()], 1.0);
        EXPECT_EQ(input[cr.sample_index() + 10], -1.0);
        
    }
    
    {
        auto cr = mobilinkd::ClockRecovery<float, 48000, 4800>();
        for (size_t i = 0; i != 48; ++i) // Half of M17 preamble.
        {
            for (size_t j = 0; j != input.size(); ++j)
            {
                cr(input[j]);
            }
        }
        cr.update();
        auto prev_sample_index = cr.sample_index();
        for (size_t i = 0; i != 48; ++i) // Half of M17 preamble.
        {
            for (size_t j = 0; j != input.size(); ++j)
            {
                cr(input[j]);
            }
        }
        cr.update();
        EXPECT_EQ(cr.sample_index(), prev_sample_index);

        EXPECT_EQ(cr.clock_estimate(), 1.0);
        EXPECT_EQ(int(cr.sample_index()), 5);
        // Verify we get actual best sample points.
        EXPECT_EQ(input[cr.sample_index()], 1.0);
        EXPECT_EQ(input[cr.sample_index() + 10], -1.0);
        
    }
}

TEST_F(ClockRecoveryTest, all_phases)
{
    // 2400Hz sine wave -- same as M17 preamble.
    constexpr std::array<double, 20> input = {
        0.0, 0.30901699437494745, 0.5877852522924732, 0.8090169943749475,
        0.9510565162951536, 1.0, 0.9510565162951536, 0.8090169943749475,
        0.5877852522924728, 0.3090169943749471, 0.0, -0.30901699437494773,
        -0.587785252292473, -0.8090169943749479, -0.9510565162951535, -1.0,
        -0.9510565162951533, -0.8090169943749476, -0.5877852522924726, -0.3090169943749468
    };

    std::array<uint8_t, 10> expected = {5, 4, 3, 2, 1, 0, 9, 8, 7, 6};
    
    for (size_t p = 0; p != 10; ++p)
    {
        auto cr = mobilinkd::ClockRecovery<float, 48000, 4800>();
        for (size_t i = 0; i != 48; ++i) // Half of M17 preamble.
        {
            for (size_t j = 0; j != input.size(); ++j)
            {
                auto index = (j + p) % 10;
                cr(input[index]);
            }
        }

        cr.update();
        EXPECT_EQ(cr.clock_estimate(), 1.0);
        EXPECT_EQ(int(cr.sample_index()), expected[p]);
    }
}
