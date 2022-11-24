#include "LinkSetupFrame.h"

#include <gtest/gtest.h>

#include <cstdint>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class LinkSetupFrameTest : public ::testing::Test {
 protected:
  // void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(LinkSetupFrameTest, encode_callsign)
{
    mobilinkd::LinkSetupFrame::call_t callsign = {'W', 'X', '9', 'O'};
    auto encoded = mobilinkd::LinkSetupFrame::encode_callsign(callsign);
    EXPECT_EQ(encoded[0], 0);
    EXPECT_EQ(encoded[1], 0);
    EXPECT_EQ(encoded[2], 0);
    EXPECT_EQ(encoded[3], 0x0f);
    EXPECT_EQ(encoded[4], 0x8a);
    EXPECT_EQ(encoded[5], 0xd7);
}

TEST_F(LinkSetupFrameTest, decode_callsign)
{
    mobilinkd::LinkSetupFrame::encoded_call_t encoded = {0, 0, 0, 0x0f, 0x8a, 0xd7};
    auto callsign = mobilinkd::LinkSetupFrame::decode_callsign(encoded);
    EXPECT_EQ(callsign[0], 'W');
    EXPECT_EQ(callsign[1], 'X');
    EXPECT_EQ(callsign[2], '9');
    EXPECT_EQ(callsign[3], 'O');
    EXPECT_EQ(callsign[4], 0);
}

TEST_F(LinkSetupFrameTest, decode_callsign_2)
{
    mobilinkd::LinkSetupFrame::encoded_call_t encoded = {0x00, 0x00, 0x5F, 0x1B, 0x66, 0x91};
    auto callsign = mobilinkd::LinkSetupFrame::decode_callsign(encoded);
    EXPECT_EQ(callsign[0], 'I') << (char*) callsign.data();
    EXPECT_EQ(callsign[1], 'U');
    EXPECT_EQ(callsign[2], '2');
    EXPECT_EQ(callsign[3], 'K');
    EXPECT_EQ(callsign[4], 'W');
    EXPECT_EQ(callsign[5], 'O');
    EXPECT_EQ(callsign[6], 0);
}

