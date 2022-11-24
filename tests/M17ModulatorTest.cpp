#include "M17Modulator.h"

#include <gtest/gtest.h>

#include <cstdint>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class M17ModulatorTest : public ::testing::Test {
 protected:
  void SetUp() override { }

  // void TearDown() override {}

};

TEST_F(M17ModulatorTest, construct)
{
    std::array<int16_t, 320> zeros;
    zeros.fill(0);
    mobilinkd::M17Modulator mod("W1AW");
}

TEST_F(M17ModulatorTest, run)
{
    using namespace mobilinkd;

    std::array<int16_t, 320> zeros;
    zeros.fill(0);
    M17Modulator mod("W1AW");
    auto audio_queue = std::make_shared<M17Modulator::audio_queue_t>();
    auto bitstream_queue = std::make_shared<M17Modulator::bitstream_queue_t>();
    auto future = mod.run(audio_queue, bitstream_queue);
    ASSERT_EQ(mod.state(), M17Modulator::State::IDLE);
    audio_queue->close();
    future.get();
    bitstream_queue->close();
}

