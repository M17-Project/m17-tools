#include "Util.h"

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class UtilTest : public ::testing::Test {
 protected:
  void SetUp() override { }

  // void TearDown() override {}

};

/*
 * Bit index go from 0 to N for an array of M bytes, where N is
 * from M*8-7 to M*8.  Bits within bytes are accessed from high
 * bit to low bit as byte arrays are serialized MSB first.
 */
TEST_F(UtilTest, get_bit_index)
{
    using mobilinkd::get_bit_index;

    std::array<uint8_t, 1> data = {0x55};
    EXPECT_EQ(get_bit_index(data, 0), 0);
    EXPECT_EQ(get_bit_index(data, 1), 1);
    EXPECT_EQ(get_bit_index(data, 2), 0);
    EXPECT_EQ(get_bit_index(data, 3), 1);
    EXPECT_EQ(get_bit_index(data, 4), 0);
    EXPECT_EQ(get_bit_index(data, 5), 1);
    EXPECT_EQ(get_bit_index(data, 6), 0);
    EXPECT_EQ(get_bit_index(data, 7), 1);
}

TEST_F(UtilTest, set_bit_index)
{
    using mobilinkd::get_bit_index;
    using mobilinkd::set_bit_index;

    std::array<uint8_t, 1> data = {0x55};

    EXPECT_EQ(get_bit_index(data, 0), 0);

    set_bit_index(data, 0);

    EXPECT_EQ(get_bit_index(data, 0), 1);
    EXPECT_EQ(data[0], 0xD5);
}

TEST_F(UtilTest, reset_bit_index)
{
    using mobilinkd::get_bit_index;
    using mobilinkd::reset_bit_index;

    std::array<uint8_t, 1> data = {0x55};

    EXPECT_EQ(get_bit_index(data, 7), 1);

    reset_bit_index(data, 7);

    EXPECT_EQ(get_bit_index(data, 7), 0);
    EXPECT_EQ(data[0], 0x54);
}

TEST_F(UtilTest, assign_bit_index)
{
    using mobilinkd::get_bit_index;
    using mobilinkd::assign_bit_index;

    std::array<uint8_t, 1> data = {0x55};

    EXPECT_EQ(get_bit_index(data, 2), 0);
    EXPECT_EQ(get_bit_index(data, 5), 1);

    assign_bit_index(data, 2, 1);
    assign_bit_index(data, 5, 0);

    EXPECT_EQ(get_bit_index(data, 2), 1);
    EXPECT_EQ(get_bit_index(data, 5), 0);
    EXPECT_EQ(data[0], 0x71);
}

TEST_F(UtilTest, puncture_bytes)
{
    using mobilinkd::puncture_bytes;
    using mobilinkd::get_bit_index;

    std::array<uint8_t, 8> data = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
    std::array<int8_t, 8> PM = {1,1,1,1,1,1,1,0};
    std::array<uint8_t, 7> out;

    out.fill(0);
    auto result = puncture_bytes(data, out, PM);

    EXPECT_EQ(result, 56);
    EXPECT_EQ(get_bit_index(out, 48), 0);
    EXPECT_EQ(get_bit_index(out, 49), 0);
    EXPECT_EQ(get_bit_index(out, 50), 1);
    EXPECT_EQ(get_bit_index(out, 51), 0);
    EXPECT_EQ(get_bit_index(out, 52), 1);
    EXPECT_EQ(get_bit_index(out, 53), 0);
    EXPECT_EQ(get_bit_index(out, 54), 1);
    EXPECT_EQ(get_bit_index(out, 55), 0);
}

TEST_F(UtilTest, llr_size)
{
    auto s = mobilinkd::detail::llr_size<4>();
    EXPECT_EQ(s, 43) << "size = " << s;
}

TEST_F(UtilTest, llr_not_zero)
{
    for (float i = -4.0; i < 4.0; i += 0.1)
    {
        auto [a, b] = mobilinkd::llr<float, 4>(i);
        EXPECT_NE(int(a), 0) << i << ", a = " << int(a);
        EXPECT_NE(int(b), 0) << i << ", b = " << int(b);
    }
}

TEST_F(UtilTest, llr_near_zero)
{
    {
        float v = 0.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), -1) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
    {
        float v = -0.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), 1) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
}

TEST_F(UtilTest, llr_near_one)
{
    {
        float v = 1.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), -7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
    {
        float v = 0.9999;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), -7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
}

TEST_F(UtilTest, llr_near_two)
{
    {
        float v = 2.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), -7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), 1) << v << ", b = " << int(b);
    }
    {
        float v = 1.9999;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), -7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -1) << v << ", b = " << int(b);
    }
}

TEST_F(UtilTest, llr_near_minus_one)
{
    {
        float v = -1.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), 7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
    {
        float v = -0.9999;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), 7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -7) << v << ", b = " << int(b);
    }
}

TEST_F(UtilTest, llr_near_minus_two)
{
    {
        float v = -2.0001;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), 7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), 1) << v << ", b = " << int(b);
    }
    {
        float v = -1.9999;
        auto [a, b] = mobilinkd::llr<float, 4>(v);
        EXPECT_EQ(int(a), 7) << v << ", a = " << int(a);
        EXPECT_EQ(int(b), -1) << v << ", b = " << int(b);
    }
}

TEST_F(UtilTest, PRBS9)
{
    mobilinkd::PRBS9 prbs;
    uint16_t lfsr = 0x100;

    for (size_t i = 0; i != 511; ++i) {
        lfsr = ((std::popcount(lfsr & 0x11u) & 1) << 8) | (lfsr >> 1);
        bool p = (lfsr & 0x100) == 0x100;
        bool n = prbs.generate();
        EXPECT_EQ(p,n) << "i = " << i;
    }
}

TEST_F(UtilTest, BERT_first_frame)
{
    mobilinkd::PRBS9 prbs;
    bool baseline[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1,
        0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1,
        0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1,
        0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1,
        0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0,
        0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1,
        0, 0, 0, 1, 1, 1, 1, 1};

    for (size_t i = 0; i != 197; ++i) {
        bool n = prbs.generate();
        EXPECT_EQ(baseline[i + 8],n) << i;
    }
}
#include <iomanip>

TEST_F(UtilTest, PRBS9_FULL)
{
    mobilinkd::PRBS9 prbs_generator;
    mobilinkd::PRBS9 prbs_validator;

    uint16_t byte = 0;
    uint16_t bits = 0;
    for (size_t i = 0; i != 1000; ++i) {
        bool n = prbs_generator.generate();
        byte <<= 1;
        byte |= n;
        if (++bits == 8) {
            std::cout << std::hex << std::setw(2) << byte << " ";
            byte = 0;
            bits = 0;
        }

        if (i == 499) n = !n;
        else if (i == 510) n = !n;
        prbs_validator.validate(n);
    }

    std::cout << std::endl;

    ASSERT_TRUE(prbs_validator.sync());
    EXPECT_EQ(prbs_validator.bits(), 1000);
    EXPECT_EQ(prbs_validator.errors(), 2);
}
