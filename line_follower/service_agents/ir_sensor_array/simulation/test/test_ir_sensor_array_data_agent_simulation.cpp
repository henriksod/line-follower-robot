// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>

#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/service_agents/ir_sensor_array/ir_sensor_array_data_agent.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/types/line.h"
#include "line_follower/types/pose.h"
#include "line_follower/types/track_lines.h"

namespace line_follower {
namespace {
constexpr double kMillimetersToMeters{0.001};

constexpr IrSensorData white{true, kMaximumTrackLineWhiteness};
constexpr IrSensorData black{false, 0U};
constexpr double kLedArraySpacingMilli{4.0};
constexpr double kLedArraySpacingMeters{kLedArraySpacingMilli * kMillimetersToMeters};
constexpr double kNumberOfLeds{15U};
constexpr double k{0.0};
constexpr double kLeftmostLedPositionX{-0.5 * kNumberOfLeds * kLedArraySpacingMeters};
constexpr double kRightmostLedPositionX{0.5 * kNumberOfLeds * kLedArraySpacingMeters};
constexpr double kMiddleLedPositionX{0.0};

std::vector<TrackLineSegment> input_track_line_list = {
    {{{kLeftmostLedPositionX, 1.0, 0.0}, {kLeftmostLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kRightmostLedPositionX, 1.0, 0.0}, {kRightmostLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, kLedArraySpacingMeters / 3.0, 0U, true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters / 3.0,
     0U,
     false},
    {{{kLeftmostLedPositionX, 0.0, 0.0}, {kRightmostLedPositionX, 0.0, 0.0}},
     kLedArraySpacingMeters / 3.0,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters * 0.5,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters * 1.5,
     0U,
     true},
    {{{kMiddleLedPositionX, 1.0, 0.0}, {kMiddleLedPositionX, -1.0, 0.0}},
     kLedArraySpacingMeters* kNumberOfLeds,
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
    auto iter1 = input_track_line_list.begin();
    auto iter2 = expected_activated_leds_list.begin();

    while (iter1 != input_track_line_list.end() && iter2 != expected_activated_leds_list.end()) {
        TrackLineSegment segment{*iter1};
        auto expected_output{*iter2};

        constexpr uint32_t kUpdateIntervalMicros{10000U};
        IrSensorArrayData received_ir_sensor_array_data{};

        Pose ir_sensor_array_pose_in_track_segment{};

        TrackSegment track_segment{};
        track_segment.track_lines[0] = segment;
        ir_sensor_array_model_->setTrackLines(track_segment, ir_sensor_array_pose_in_track_segment);

        bool receiveDataWasCalled{false};
        IrSensorArrayDataConsumerAgent ir_sensor_array_data_consumer_agent{};
        ir_sensor_array_data_consumer_agent.onReceiveData(
            [&](IrSensorArrayData const& ir_sensor_array_data) {
                for (std::size_t idx{0U}; idx < kNumberOfLeds; ++idx) {
                    EXPECT_EQ(ir_sensor_array_data.ir_sensor_readings[idx].detected_white_surface,
                              expected_output[idx].detected_white_surface);
                    EXPECT_EQ(ir_sensor_array_data.ir_sensor_readings[idx].digital_reading,
                              expected_output[idx].digital_reading);
                }

                receiveDataWasCalled = true;
                received_ir_sensor_array_data = ir_sensor_array_data;
            });
        ir_sensor_array_data_consumer_agent.attach(*ir_sensor_array_data_producer_agent_);

        ir_sensor_array_data_producer_agent_->schedule(scheduler_, kUpdateIntervalMicros);

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
