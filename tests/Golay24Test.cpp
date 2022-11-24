#include "Golay24.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <bitset>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

class Golay24Test : public ::testing::Test {
protected:
    void SetUp() override {}

    // void TearDown() override {}
};

TEST_F(Golay24Test, encode24)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    auto result = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(result, 0xD7880FU);
}

TEST_F(Golay24Test, decode24)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    uint32_t decoded = 0;
    EXPECT_TRUE(mobilinkd::Golay24::decode(encoded, decoded));
    EXPECT_EQ(decoded, 0xD7880FU);
}

TEST_F(Golay24Test, decode_corrupted_1)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    uint32_t corruption = 0x010000;
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ corruption;
    uint32_t decoded = 0;
    EXPECT_TRUE(mobilinkd::Golay24::decode(corrupted, decoded));
    EXPECT_EQ(decoded, 0xD7880FU);
}

TEST_F(Golay24Test, decode_corrupted_2)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    uint32_t corruption = 0x010010;
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ corruption;
    uint32_t decoded = 0;
    EXPECT_TRUE(mobilinkd::Golay24::decode(corrupted, decoded));
    EXPECT_EQ(decoded, 0xD7880FU);
}

TEST_F(Golay24Test, decode_corrupted_3)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    uint32_t corruption = 0x810100;
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ corruption;
    uint32_t decoded = 0;
    EXPECT_TRUE(mobilinkd::Golay24::decode(corrupted, decoded));
    EXPECT_EQ(decoded, 0xD7880FU);
}

TEST_F(Golay24Test, decode_corrupted_4)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    uint32_t corruption = 0x011110;
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ corruption;
    uint32_t decoded = 0;
    EXPECT_FALSE(mobilinkd::Golay24::decode(corrupted, decoded));
}

TEST_F(Golay24Test, syndrome_map)
{
    uint32_t corruption = 0x010010;
    auto c_syndrome = mobilinkd::Golay24::syndrome(corruption);
    uint16_t link_setup_group = 0b110101111000; // 0xD78
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ (corruption << 1);
    auto d_syndrome = mobilinkd::Golay24::syndrome(corrupted >> 1);
    EXPECT_EQ(c_syndrome, d_syndrome);
}


TEST_F(Golay24Test, decode_map)
{
    // uint16_t data = 0xD78;
    uint16_t link_setup_group = 0b110101111000; // 0xD78, 3448
    uint32_t corruption = 0x010010;
    auto encoded = mobilinkd::Golay24::encode24(link_setup_group);
    EXPECT_EQ(encoded, 0xD7880FU);
    auto corrupted = encoded ^ corruption;
    uint32_t decoded = 0;
    EXPECT_TRUE(mobilinkd::Golay24::decode(corrupted, decoded));
    EXPECT_EQ(decoded >> 12, link_setup_group);

#if 0
    size_t c = 0;
    for (auto x : mobilinkd::Golay24::LUT) {
        std::cout << std::hex << std::setfill('0') << std::setw(8) << x.a << ":" << std::setw(4) << x.b;
        if (c++ == 7) {
            c = 0;
            std::cout << std::endl;
        } else std::cout << ", ";
    }
#endif
}

/**
 * Test interop between C++ & Python implementations
 */
TEST_F(Golay24Test, interop)
{
    std::array<uint32_t, 4> encoded = {
        0b110101111000100000001111U,
        0b101000001111010110011001U,
        0b000000000000000000000000U,
        0b000000000001100011101011U
    };
    
    std::array<uint32_t, 4> expected = {
        0b110101111000,
        0b101000001111,
        0b000000000000,
        0b000000000001
    };
    
    for (size_t i = 0; i != encoded.size(); ++i)
    {
        uint32_t decoded;
        EXPECT_TRUE(mobilinkd::Golay24::decode(encoded[i], decoded));
        EXPECT_EQ(decoded >> 12, expected[i]);
    }
}