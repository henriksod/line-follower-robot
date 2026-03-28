// Copyright (c) 2024 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/line_following/simple_line_following_model.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
namespace {

void resetIrSensorArrayData(IrSensorArrayData& ir_sensor_array_data) {
    ir_sensor_array_data.valid = false;
    for (auto& ir_sensor_data : ir_sensor_array_data.ir_sensor_readings) {
        ir_sensor_data.detected_white_surface = true;
        ir_sensor_data.intensity = 1.0;
    }
}

DifferentialDriveRobotCharacteristics createRobotCharacteristics() {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = 0.02;
    robot_characteristics.distance_between_wheels = 0.1;
    return robot_characteristics;
}

LineFollowingCharacteristics createLineFollowingCharacteristics() {
    LineFollowingCharacteristics line_following_characteristics{};

    line_following_characteristics.pid_speed_parameters.proportional_gain = 2.0;
    line_following_characteristics.pid_speed_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.min_value = 0.0;
    line_following_characteristics.pid_speed_parameters.max_value = 100.0;

    line_following_characteristics.pid_steer_parameters.proportional_gain = 4.0;
    line_following_characteristics.pid_steer_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.min_value = -2.0;
    line_following_characteristics.pid_steer_parameters.max_value = 2.0;

    line_following_characteristics.max_forward_velocity = 1.0;
    line_following_characteristics.measurement_noise = 0.0;

    line_following_characteristics.sharp_turn_forward_velocity = 0.1;
    line_following_characteristics.sharp_turn_angular_velocity = 2.0;

    // New fields – use tight timeout so tests finish quickly
    line_following_characteristics.lost_line_timeout_seconds = 0.01;
    line_following_characteristics.perpendicular_crossing_forward_velocity = 0.2;
    line_following_characteristics.perpendicular_crossing_duration_seconds = 0.001;
    line_following_characteristics.position_filter_alpha = 1.0;
    line_following_characteristics.sharp_turn_detection_ratio = 1.0 / 3.0;

    return line_following_characteristics;
}

/// Helper: build a LineFollowingContext with all required states.
std::unique_ptr<detail::LineFollowingContext> makeContext(
    LineFollowingCharacteristics const& chars, DeadReckoningModel& drm) {
    return std::make_unique<detail::LineFollowingContext>(
        chars, drm, std::make_unique<detail::StartState>(),
        std::make_unique<detail::LineTrackingState>(),
        std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
        std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
        std::make_unique<detail::LostLineState>(),
        std::make_unique<detail::PerpendicularCrossingState>(),
        std::make_unique<detail::StopState>());
}

TEST(SimpleLineFollowingModelTest, TestPredictNoInput) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        makeContext(createLineFollowingCharacteristics(), *dead_reckoning_model_ptr)};
    detail::LineFollowingContext* context_ptr = line_following_context.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model),
                                   std::move(line_following_context));

    model.update(SystemTime{0UL});
    EXPECT_TRUE(context_ptr->start_state->is_active());

    double const expected_speed{dead_reckoning_model_ptr->calculateLeftMotorSpeed(0.1, 0.0)};
    EXPECT_NEAR(model.getMotorSignalLeft().speed.revolutions_per_second, expected_speed, 1e-6);
    EXPECT_NEAR(model.getMotorSignalRight().speed.revolutions_per_second, expected_speed, 1e-6);
}

TEST(SimpleLineFollowingModelTest, TestSteersRightWhenLineIsRight) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        makeContext(createLineFollowingCharacteristics(), *dead_reckoning_model_ptr)};
    detail::LineFollowingContext* context_ptr = line_following_context.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model),
                                   std::move(line_following_context));

    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    // Simulate black line detected on the right side
    ir_data.ir_sensor_readings.at(12U) = {false, 0.0};

    Pose const& pose_before = model.getPose();
    EXPECT_EQ(pose_before.position.y, 0.0);
    EXPECT_EQ(pose_before.rotation.z, 0.0);

    EXPECT_TRUE(context_ptr->start_state->is_active());

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});  // Start state
    EXPECT_FALSE(context_ptr->start_state->is_active());
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());
    ir_data.timestamp = SystemTime{3000UL};

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});  // Line tracking state

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(SystemTime{5000UL});  // Line tracking state
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());

    double left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(left, 0.0);
    EXPECT_GT(right, 0.0);
    EXPECT_GT(left, right);  // steer right

    Pose const& pose_after = model.getPose();
    EXPECT_GT(pose_after.position.x, 0.0);
    EXPECT_LT(pose_after.position.y, 0.0);
    EXPECT_LT(pose_after.rotation.z, 0.0);
}

TEST(SimpleLineFollowingModelTest, TestSteersLeftWhenLineIsLeft) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        makeContext(createLineFollowingCharacteristics(), *dead_reckoning_model_ptr)};
    detail::LineFollowingContext* context_ptr = line_following_context.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model),
                                   std::move(line_following_context));

    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    // Simulate black line detected on the left side
    ir_data.ir_sensor_readings.at(2U) = {false, 0.0};

    Pose const& pose_before = model.getPose();
    EXPECT_EQ(pose_before.position.y, 0.0);
    EXPECT_EQ(pose_before.rotation.z, 0.0);

    EXPECT_TRUE(context_ptr->start_state->is_active());

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});  // Start state

    EXPECT_FALSE(context_ptr->start_state->is_active());
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());
    ir_data.timestamp = SystemTime{3000UL};

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});  // Line tracking state

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(SystemTime{5000UL});  // Line tracking state
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());

    double left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(left, 0.0);
    EXPECT_GT(right, 0.0);
    EXPECT_GT(right, left);  // steer left

    Pose const& pose_after = model.getPose();
    EXPECT_GT(pose_after.position.x, 0.0);
    EXPECT_GT(pose_after.position.y, 0.0);
    EXPECT_GT(pose_after.rotation.z, 0.0);
}

TEST(SimpleLineFollowingModelTest, TestDrivesStraightWhenLineIsCentered) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        makeContext(createLineFollowingCharacteristics(), *dead_reckoning_model_ptr)};
    detail::LineFollowingContext* context_ptr = line_following_context.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model),
                                   std::move(line_following_context));

    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    // Simulate black line detected in the center
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    Pose const& pose_before = model.getPose();
    EXPECT_EQ(pose_before.position.y, 0.0);
    EXPECT_EQ(pose_before.rotation.z, 0.0);

    EXPECT_TRUE(context_ptr->start_state->is_active());
    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});  // Start state

    EXPECT_FALSE(context_ptr->start_state->is_active());
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());
    ir_data.timestamp = SystemTime{3000UL};
    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});  // Line tracking state

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(SystemTime{5000UL});  // Line tracking state
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());

    double left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(left, 0.0);
    EXPECT_GT(right, 0.0);
    EXPECT_NEAR(left, right, 1e-6);  // go straight

    Pose const& pose_after = model.getPose();
    EXPECT_GT(pose_after.position.x, 0.0);
    EXPECT_EQ(pose_after.position.y, 0.0);
    EXPECT_EQ(pose_after.rotation.z, 0.0);
}

TEST(SimpleLineFollowingModelTest, TestSharpTurnLeftWhenLineIsLeft) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        makeContext(createLineFollowingCharacteristics(), *dead_reckoning_model_ptr)};
    detail::LineFollowingContext* context_ptr = line_following_context.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model),
                                   std::move(line_following_context));

    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    // Simulate black line detected in the center
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    Pose const& pose_before = model.getPose();
    EXPECT_EQ(pose_before.position.y, 0.0);
    EXPECT_EQ(pose_before.rotation.z, 0.0);

    EXPECT_TRUE(context_ptr->start_state->is_active());
    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});  // Start state

    EXPECT_FALSE(context_ptr->start_state->is_active());
    EXPECT_TRUE(context_ptr->line_tracking_state->is_active());
    ir_data.timestamp = SystemTime{3000UL};
    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});  // Line tracking state

    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{5000UL};
    ir_data.number_of_leds = 15U;
    // Simulate black line at 90 degrees to the left
    for (size_t idx{0U}; idx < ir_data.number_of_leds / 2U; ++idx) {
        ir_data.ir_sensor_readings[idx].detected_white_surface = false;
        ir_data.ir_sensor_readings[idx].intensity = 0.0;
    }

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});  // Sharp turn left state
    EXPECT_TRUE(context_ptr->sharp_turn_left_state->is_active());

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(SystemTime{7000UL});  // Sharp turn left state
    EXPECT_TRUE(context_ptr->sharp_turn_left_state->is_active());

    double left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(right, left);

    Pose const& pose_after = model.getPose();
    EXPECT_GT(pose_after.position.x, 0.0);
    EXPECT_GT(pose_after.position.y, 0.0);
    EXPECT_GT(pose_after.rotation.z, 0.0);
}

// ---- Phase 1: LostLineState ----

TEST(SimpleLineFollowingModelTest, TestTransitionsToLostLineStateWhenNoLine) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{
        makeContext(createLineFollowingCharacteristics(), *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Step 1: boot into LineTracking via a centred line
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    ir_data.timestamp = SystemTime{3000UL};
    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});

    // Step 2: remove all line detections
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{5000UL};
    ir_data.number_of_leds = 15U;

    model.setEncoderLeftData({model.getMotorSignalLeft().speed.revolutions_per_second, 0});
    model.setEncoderRightData({model.getMotorSignalRight().speed.revolutions_per_second, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});

    EXPECT_TRUE(ctx_ptr->lost_line_state->is_active());
}

TEST(SimpleLineFollowingModelTest, TestLostLineStateTransitionsToStopAfterTimeout) {
    auto chars = createLineFollowingCharacteristics();
    chars.lost_line_timeout_seconds = 0.001;  // 1 ms – expires after two 1000 µs steps

    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{makeContext(chars, *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    ir_data.timestamp = SystemTime{3000UL};
    model.update(ir_data);
    model.update(SystemTime{4000UL});

    // All LEDs white → no line
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;

    // Enter LostLineState
    ir_data.timestamp = SystemTime{5000UL};
    model.update(ir_data);
    model.update(SystemTime{6000UL});
    EXPECT_TRUE(ctx_ptr->lost_line_state->is_active());

    // Advance past the 1 ms timeout (2000 µs elapsed since lost_line_timestamp = 5000 µs)
    ir_data.timestamp = SystemTime{7000UL};
    model.update(ir_data);
    model.update(SystemTime{8000UL});

    EXPECT_TRUE(ctx_ptr->stop_state->is_active());
}

TEST(SimpleLineFollowingModelTest, TestLostLineStateReturnsToTrackingWhenLineFound) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{
        makeContext(createLineFollowingCharacteristics(), *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    ir_data.timestamp = SystemTime{3000UL};
    model.update(ir_data);
    model.update(SystemTime{4000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    // Lose the line
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{5000UL};
    model.update(ir_data);
    model.update(SystemTime{6000UL});
    EXPECT_TRUE(ctx_ptr->lost_line_state->is_active());

    // Re-acquire the line in the centre (within the 10 ms timeout)
    ir_data.timestamp = SystemTime{7000UL};
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};
    model.update(ir_data);
    model.update(SystemTime{8000UL});

    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());
}

// ---- Phase 3: PerpendicularCrossingState ----

TEST(SimpleLineFollowingModelTest, TestTransitionsToPerpendicularCrossingState) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{
        makeContext(createLineFollowingCharacteristics(), *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    ir_data.timestamp = SystemTime{3000UL};
    model.update(ir_data);
    model.update(SystemTime{4000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    // Simulate perpendicular line (all LEDs detect black)
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{5000UL};
    for (size_t i = 0; i < ir_data.number_of_leds; ++i) {
        ir_data.ir_sensor_readings[i] = {false, 0.0};
    }

    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});

    EXPECT_TRUE(ctx_ptr->perpendicular_crossing_state->is_active());
}

TEST(SimpleLineFollowingModelTest, TestPerpendicularCrossingStateGoesForward) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{
        makeContext(createLineFollowingCharacteristics(), *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking then trigger perpendicular crossing
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    ir_data.timestamp = SystemTime{3000UL};
    model.update(ir_data);
    model.update(SystemTime{4000UL});

    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{5000UL};
    for (size_t i = 0; i < ir_data.number_of_leds; ++i) {
        ir_data.ir_sensor_readings[i] = {false, 0.0};
    }
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});
    EXPECT_TRUE(ctx_ptr->perpendicular_crossing_state->is_active());

    // Check that both motors are commanded forward (equal speeds, > 0)
    double const left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double const right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(left, 0.0);
    EXPECT_GT(right, 0.0);
    EXPECT_NEAR(left, right, 1e-6);
}

TEST(SimpleLineFollowingModelTest, TestPerpendicularCrossingStateReturnsToTracking) {
    auto chars = createLineFollowingCharacteristics();
    chars.perpendicular_crossing_duration_seconds = 0.001;  // 1 ms

    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{makeContext(chars, *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    ir_data.timestamp = SystemTime{3000UL};
    model.update(ir_data);
    model.update(SystemTime{4000UL});

    // Enter perpendicular crossing at t=5000 µs
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{5000UL};
    for (size_t i = 0; i < ir_data.number_of_leds; ++i) {
        ir_data.ir_sensor_readings[i] = {false, 0.0};
    }
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});
    EXPECT_TRUE(ctx_ptr->perpendicular_crossing_state->is_active());

    // Advance past the 1 ms duration (2000 µs elapsed → ≥ 1000 µs = 0.001 s)
    ir_data.timestamp = SystemTime{7000UL};
    model.update(ir_data);
    model.update(SystemTime{8000UL});

    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());
}

// ---- Phase 2: Velocity clamping ----

TEST(SimpleLineFollowingModelTest, TestVelocityNotNegativeUnderExtremePosition) {
    // Set a large turning_speed_ratio (> 1) to potentially produce a negative speed.
    auto chars = createLineFollowingCharacteristics();
    chars.turning_speed_ratio = 2.0;

    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* drm = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> ctx{makeContext(chars, *drm)};
    detail::LineFollowingContext* ctx_ptr = ctx.get();
    SimpleLineFollowingModel model(std::move(dead_reckoning_model), std::move(ctx));

    // Boot into LineTracking with centred line
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{2000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    ir_data.timestamp = SystemTime{3000UL};
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{4000UL});

    // Present extreme right-edge line (only the rightmost LED active)
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{5000UL};
    ir_data.ir_sensor_readings.at(14U) = {false, 0.0};
    model.setEncoderLeftData({0, 0});
    model.setEncoderRightData({0, 0});
    model.update(ir_data);
    model.update(SystemTime{6000UL});
    EXPECT_TRUE(ctx_ptr->line_tracking_state->is_active());

    // Neither motor should be commanded backwards
    double const left = model.getMotorSignalLeft().speed.revolutions_per_second;
    double const right = model.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GE(left, 0.0);
    EXPECT_GE(right, 0.0);
}

// ---- Phase 3: position_filter_alpha (EMA smoothing) ----

TEST(SimpleLineFollowingModelTest, TestEmaFilterSmoothesPositionEstimate) {
    // alpha = 0.5: the filtered position after one step from 0 should be half the raw.
    auto chars = createLineFollowingCharacteristics();
    chars.position_filter_alpha = 0.5;
    // Disable steer PID so motor outputs directly reflect the filtered position
    chars.pid_steer_parameters.proportional_gain = 0.0;
    chars.pid_steer_parameters.integral_gain = 0.0;
    chars.pid_steer_parameters.derivative_gain = 0.0;
    chars.pid_speed_parameters.proportional_gain = 1.0;

    std::unique_ptr<DeadReckoningModel> drm_unfiltered{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    std::unique_ptr<DeadReckoningModel> drm_filtered{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};

    DeadReckoningModel* drm_unfiltered_ptr = drm_unfiltered.get();
    DeadReckoningModel* drm_filtered_ptr = drm_filtered.get();

    auto chars_no_filter = chars;
    chars_no_filter.position_filter_alpha = 1.0;

    std::unique_ptr<detail::LineFollowingContext> ctx_unfiltered{
        makeContext(chars_no_filter, *drm_unfiltered_ptr)};
    std::unique_ptr<detail::LineFollowingContext> ctx_filtered{
        makeContext(chars, *drm_filtered_ptr)};

    SimpleLineFollowingModel model_unfiltered(std::move(drm_unfiltered),
                                              std::move(ctx_unfiltered));
    SimpleLineFollowingModel model_filtered(std::move(drm_filtered), std::move(ctx_filtered));

    // Boot both into LineTracking
    IrSensorArrayData ir_data{};
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.timestamp = SystemTime{1000UL};
    ir_data.number_of_leds = 15U;
    ir_data.ir_sensor_readings.at(7U) = {false, 0.0};

    for (auto* m : {&model_unfiltered, &model_filtered}) {
        m->setEncoderLeftData({0, 0});
        m->setEncoderRightData({0, 0});
        m->update(ir_data);
        m->update(SystemTime{2000UL});
        m->update(ir_data);
        m->update(SystemTime{3000UL});
    }

    // Jump the line far right (LED 12)
    resetIrSensorArrayData(ir_data);
    ir_data.valid = true;
    ir_data.number_of_leds = 15U;
    ir_data.timestamp = SystemTime{4000UL};
    ir_data.ir_sensor_readings.at(12U) = {false, 0.0};

    for (auto* m : {&model_unfiltered, &model_filtered}) {
        m->setEncoderLeftData({0, 0});
        m->setEncoderRightData({0, 0});
        m->update(ir_data);
        m->update(SystemTime{5000UL});
    }

    // The EMA-filtered model should steer less aggressively than the unfiltered one
    double const diff_unfiltered = model_unfiltered.getMotorSignalLeft().speed.revolutions_per_second -
                                   model_unfiltered.getMotorSignalRight().speed.revolutions_per_second;
    double const diff_filtered = model_filtered.getMotorSignalLeft().speed.revolutions_per_second -
                                 model_filtered.getMotorSignalRight().speed.revolutions_per_second;
    EXPECT_GT(std::abs(diff_unfiltered), std::abs(diff_filtered));
}

}  // namespace
}  // namespace line_follower
