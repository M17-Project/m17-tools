#include "Convolution.h"
#include "Util.h"

#include <gtest/gtest.h>

#include <cstdint>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/ConvolutionTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class ConvolutionTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(ConvolutionTest, convolve)
{
    constexpr size_t K = 4;

    auto memory = mobilinkd::update_memory<K>(0, 0);
    EXPECT_EQ(mobilinkd::convolve_bit(031, memory), 0);
    EXPECT_EQ(mobilinkd::convolve_bit(027, memory), 0);

    memory = mobilinkd::update_memory<K>(0, 1);
    EXPECT_EQ(mobilinkd::convolve_bit(031, memory), 1);
    EXPECT_EQ(mobilinkd::convolve_bit(027, memory), 1);   
}

TEST_F(ConvolutionTest, convolve_bit)
{
    std::array<uint8_t, 8> input = {1,0,1,1,0,1,1,0};
    std::array<uint8_t, 24> expected = {1,1,0,1,1,0,0,0,1,1,0,0,1,1,1,1,1,1,0,1,1,1,0,0};
    std::array<uint8_t, 24> encoded;

    constexpr size_t K = 4;

    size_t index = 0;
    uint32_t memory = 0;
    for (auto x : input)
    {
        memory = mobilinkd::update_memory<K>(memory, x);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }

    // Flush
    for (size_t i = 0; i != K; ++i)
    {
        memory = mobilinkd::update_memory<K>(memory, 0);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }
    
    std::cout << std::endl;
    for (size_t i = 0; i != encoded.size(); ++i)
    {
        EXPECT_EQ(encoded[i], expected[i]) << "i = " << i;
    }
}
