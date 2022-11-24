#include "SlidingDFT.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <functional>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/SlidingDFTTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class SlidingDFTTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(SlidingDFTTest, construct)
{
    auto dft = mobilinkd::SlidingDFT<double, 48000, 3000>();
}

TEST_F(SlidingDFTTest, dft_3000hz_square)
{
    // 3000Hz square wave
    constexpr std::array<double, 16> input = {1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1};
    std::array<std::complex<double>, 48> result;

    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 1000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 2000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 3000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 3000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 4000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 5000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 6000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 7000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 8000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 3000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 9000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 10000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 11000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 12000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 13000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 14000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 15000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        // 3000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 16000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 17000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 18000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 19000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 20000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 3000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 21000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 22000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 23000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 24000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
}

TEST_F(SlidingDFTTest, dft_3000Hz_sine)
{
    // 3000Hz sine wave
    constexpr std::array<double, 16> input = {
        0.0, 0.3826834323650898, 0.7071067811865475, 0.9238795325112867,
        1.0, 0.9238795325112868, 0.7071067811865476, 0.3826834323650903,
        1.2246467991473532e-16, -0.38268343236508967, -0.7071067811865471, -0.9238795325112865,
        -1.0, -0.923879532511287, -0.7071067811865483, -0.38268343236508956};
    std::array<std::complex<double>, 48> result;

    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 1000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 2000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 3000Hz square wave should ONLY match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 3000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 4000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 5000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 6000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 7000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 8000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 9000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 10000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 11000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 12000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 13000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 14000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 15000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 16000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 17000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 18000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 19000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 20000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 21000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 22000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 23000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 24000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
}

TEST_F(SlidingDFTTest, dft_2000Hz_square)
{
    constexpr std::array<double, 24> input = {1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    std::array<std::complex<double>, 48> result;

    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 1000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 2000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 2000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 3000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 4000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        auto dft = mobilinkd::SlidingDFT<double, 48000, 5000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_LT(mag, 1.0);
    }
    {
        // 2000Hz square wave should match here.
        auto dft = mobilinkd::SlidingDFT<double, 48000, 6000>();
        auto it = std::transform(input.begin(), input.end(), result.begin(), std::ref(dft));
        it = std::transform(input.begin(), input.end(), it, std::ref(dft));
        auto mag = std::abs(result[47]);
        EXPECT_GT(mag, 5.0);
    }
}

TEST_F(SlidingDFTTest, ndft_2000Hz_square)
{
    constexpr std::array<double, 24> input = {1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    std::array<std::complex<double>, 12> result;

    {
        auto dft = mobilinkd::NSlidingDFT<double, 48000, 48, 12>({1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000,12000});
        for (size_t i = 0; i != 48; ++i)
        {
            result = dft(input[i % 24]);
        }
        for (size_t i = 0; i != 12; ++i)
        {
            auto mag = std::abs(result[i]);
            if (i == 1 || i == 5 || i == 9) // fundamental and odd harmonics
                EXPECT_GT(mag, 5.0);
            else
                EXPECT_LT(mag, 1.0);
        }
    }   
}
