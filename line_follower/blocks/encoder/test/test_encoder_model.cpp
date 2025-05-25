// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
namespace {

constexpr EncoderTag kTestTag{EncoderTag::kLeft};
constexpr int kCountsPerRevolution = 1000;

TEST(EncoderModelTest, StartsAtZero) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);
    EncoderData data{};
    EXPECT_TRUE(model.getEncoderData(data));
    EXPECT_EQ(data.step_position, 0);
    EXPECT_DOUBLE_EQ(data.revolutions_per_second, 0.0);
}

TEST(EncoderModelTest, AccumulatesPositiveSteps) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);

    model.setRotorSpeed({1.0});  // 1 rev/sec

    model.tick(SystemTime{1'000'000});  // t = 1.0s
    model.tick(SystemTime{2'000'000});  // t = 2.0s

    EncoderData data{};
    ASSERT_TRUE(model.getEncoderData(data));
    EXPECT_EQ(data.step_position, 2000);  // 1 rev × 1000 steps
    EXPECT_DOUBLE_EQ(data.revolutions_per_second, 1.0);
}

TEST(EncoderModelTest, AccumulatesFractionalStepsOverTime) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);
    model.setRotorSpeed({0.5});  // 0.5 rev/sec

    model.tick(SystemTime{0});
    model.tick(SystemTime{2'000'000});  // 2 sec

    EncoderData data{};
    ASSERT_TRUE(model.getEncoderData(data));
    EXPECT_EQ(data.step_position, 1000);  // 0.5 * 2 = 1 rev
}

TEST(EncoderModelTest, SupportsNegativeRotorSpeed) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);
    model.setRotorSpeed({-1.0});  // reverse 1 rev/sec

    model.tick(SystemTime{1'000'000});
    model.tick(SystemTime{2'000'000});

    EncoderData data{};
    ASSERT_TRUE(model.getEncoderData(data));
    // This test will FAIL if step_position is unsigned
    EXPECT_EQ(data.step_position, -2000);
    EXPECT_DOUBLE_EQ(data.revolutions_per_second, -1.0);
}

TEST(EncoderModelTest, HandlesOutOfOrderTicksGracefully) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);
    model.setRotorSpeed({1.0});

    model.tick(SystemTime{2'000'000});
    model.tick(SystemTime{1'000'000});  // earlier timestamp, should not go negative or crash

    EncoderData data{};
    ASSERT_TRUE(model.getEncoderData(data));
    EXPECT_GE(data.step_position, 0);  // Might be unchanged if model handles it defensively
}

TEST(EncoderModelTest, PreservesSubStepAccuracyAcrossTicks) {
    EncoderModel model({kCountsPerRevolution}, kTestTag);
    model.setRotorSpeed({0.001});  // very slow speed

    model.tick(SystemTime{0});
    model.tick(SystemTime{1'000'000});  // 1 second

    EncoderData data{};
    ASSERT_TRUE(model.getEncoderData(data));
    EXPECT_EQ(data.step_position, 1);  // 0.001 rev × 1000 steps = 1 step
}

}  // namespace
}  // namespace line_follower
