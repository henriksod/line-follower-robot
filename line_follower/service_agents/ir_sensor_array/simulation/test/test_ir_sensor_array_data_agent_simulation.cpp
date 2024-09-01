// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/line.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/track_lines.h"

namespace line_follower {
namespace {
constexpr double kMillimetersToMeters{0.001};
constexpr double kMetersToMillimeters{1000.0};

constexpr IrSensorData white{true, 1.0};
constexpr IrSensorData black{false, 0.0};
constexpr double kLedArraySpacingMilli{4.0};
constexpr double kLedArraySpacingMeters{kLedArraySpacingMilli * kMillimetersToMeters};
constexpr double kNumberOfLeds{15U};
constexpr double k{0.0};
constexpr double kLeftmostLedPositionX{-0.5 * kNumberOfLeds * kLedArraySpacingMeters};
constexpr double kRightmostLedPositionX{0.5 * kNumberOfLeds * kLedArraySpacingMeters};
constexpr double kMiddleLedPositionX{0.0};

std::vector<TrackLineSegment> input_track_line_list = {
    {{{kLeftmostLedPositionX, 1.0, 0.0}, {kLeftmostLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kRightmostLedPositionX, 1.0, 0.0}, {kRightmostLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, kLedArraySpacingMeters / 3.0, 0U, true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters / 3.0,
     0U,
     false},
    {{{kLeftmostLedPositionX, 0.0, 0.0}, {kRightmostLedPositionX, 0.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters * 0.5,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters * 1.5,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kMetersToMillimeters* kLedArraySpacingMeters* kNumberOfLeds,
     0U,
     true}};
std::vector<std::array<IrSensorData, kMaxIrSensorArrayNumberOfLeds> > expected_activated_leds_list =
    {{black, white, white, white, white, white, white, white, white, white, white, white, white,
      white, white},
     {white, white, white, white, white, white, white, white, white, white, white, white, white,
      white, black},
     {white, white, white, white, white, white, white, black, white, white, white, white, white,
      white, white},
     {white, white, white, white, white, white, white, white, white, white, white, white, white,
      white, white},
     {white, white, white, white, white, white, white, white, white, white, white, white, white,
      white, white},
     {black, black, black, black, black, black, black, black, black, black, black, black, black,
      black, black},
     {white, white, white, white, white, white, white, black, white, white, white, white, white,
      white, white},
     {white, white, white, white, white, white, black, black, black, white, white, white, white,
      white, white},
     {black, black, black, black, black, black, black, black, black, black, black, black, black,
      black, black}};

Pose createInitialPose() {
    return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
}

class IrSensorArrayDataAgentTest : public ::testing::Test {
 protected:
    void SetUp() override {
        ir_sensor_array_characteristics_.array_spacing = kLedArraySpacingMilli;
        ir_sensor_array_characteristics_.number_of_leds = 15U;

        auto ir_sensor_array_model{
            std::make_unique<IrSensorArrayModel>(ir_sensor_array_characteristics_)};
        ir_sensor_array_model_ = ir_sensor_array_model.get();
        ir_sensor_array_data_producer_agent_ =
            std::make_unique<IrSensorArrayDataProducerAgent>(std::move(ir_sensor_array_model));

        scheduler_ = std::make_shared<SchedulerProducerAgent>();
    }

    void TearDown() override {}

    IrSensorArrayCharacteristics ir_sensor_array_characteristics_{};
    IrSensorArrayModel* ir_sensor_array_model_;
    std::unique_ptr<IrSensorArrayDataProducerAgent> ir_sensor_array_data_producer_agent_;
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
};

TEST_F(IrSensorArrayDataAgentTest, GetIrSensorArrayData) {
    /// TODO: Add test where we have multiple track lines and track segments
    auto iter1 = input_track_line_list.begin();
    auto iter2 = expected_activated_leds_list.begin();

    while (iter1 != input_track_line_list.end() && iter2 != expected_activated_leds_list.end()) {
        TrackLineSegment segment{*iter1};
        auto expected_output{*iter2};

        constexpr uint32_t kUpdateIntervalMicros{10000U};
        IrSensorArrayData received_ir_sensor_array_data{};

        Pose ir_sensor_array_pose{createInitialPose()};

        std::vector<TrackSegment> track_segments{};
        TrackSegment track_segment{};
        track_segment.track_lines[0] = segment;

        // Add an extra visible line to exercise multiple lines
        TrackLineSegment dummy_track_line{};
        dummy_track_line.visible = true;
        track_segment.track_lines[1] = dummy_track_line;

        track_segments.push_back(track_segment);

        // Add an extra track segment to exercise multiple segments
        TrackSegment dummy_track_segment{};
        dummy_track_segment.track_lines[0] = dummy_track_line;
        track_segments.push_back(dummy_track_segment);

        ir_sensor_array_model_->setTrackLines(track_segments, ir_sensor_array_pose);

        bool receiveDataWasCalled{false};
        IrSensorArrayDataConsumerAgent ir_sensor_array_data_consumer_agent{};
        ir_sensor_array_data_consumer_agent.onReceiveData(
            [&](IrSensorArrayData const& ir_sensor_array_data) {
                for (std::size_t idx{0U}; idx < kNumberOfLeds; ++idx) {
                    EXPECT_EQ(ir_sensor_array_data.ir_sensor_readings[idx].detected_white_surface,
                              expected_output[idx].detected_white_surface);
                    EXPECT_NEAR(ir_sensor_array_data.ir_sensor_readings[idx].intensity,
                                expected_output[idx].intensity, 1e-6);
                }

                receiveDataWasCalled = true;
                received_ir_sensor_array_data = ir_sensor_array_data;
            });
        ir_sensor_array_data_consumer_agent.attach(*ir_sensor_array_data_producer_agent_);

        ir_sensor_array_data_producer_agent_->schedule(*scheduler_, kUpdateIntervalMicros);

        auto now = std::chrono::steady_clock::now;
        using std::chrono_literals::operator""ms;
        auto work_duration = 20ms;
        auto start = now();

        while ((now() - start) < work_duration) scheduler_->tick();

        ASSERT_TRUE(receiveDataWasCalled);

        ++iter1;
        ++iter2;
    }
}
}  // namespace
}  // namespace line_follower
