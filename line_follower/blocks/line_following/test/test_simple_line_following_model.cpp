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

    return line_following_characteristics;
}

TEST(SimpleLineFollowingModelTest, TestPredictNoInput) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};
    DeadReckoningModel* dead_reckoning_model_ptr = dead_reckoning_model.get();
    std::unique_ptr<detail::LineFollowingContext> line_following_context{
        std::make_unique<detail::LineFollowingContext>(
            createLineFollowingCharacteristics(), *dead_reckoning_model_ptr,
            std::make_unique<detail::StartState>(), std::make_unique<detail::LineTrackingState>(),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
            std::make_unique<detail::StopState>())};
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
        std::make_unique<detail::LineFollowingContext>(
            createLineFollowingCharacteristics(), *dead_reckoning_model_ptr,
            std::make_unique<detail::StartState>(), std::make_unique<detail::LineTrackingState>(),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
            std::make_unique<detail::StopState>())};
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
        std::make_unique<detail::LineFollowingContext>(
            createLineFollowingCharacteristics(), *dead_reckoning_model_ptr,
            std::make_unique<detail::StartState>(), std::make_unique<detail::LineTrackingState>(),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
            std::make_unique<detail::StopState>())};
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
        std::make_unique<detail::LineFollowingContext>(
            createLineFollowingCharacteristics(), *dead_reckoning_model_ptr,
            std::make_unique<detail::StartState>(), std::make_unique<detail::LineTrackingState>(),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
            std::make_unique<detail::StopState>())};
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
        std::make_unique<detail::LineFollowingContext>(
            createLineFollowingCharacteristics(), *dead_reckoning_model_ptr,
            std::make_unique<detail::StartState>(), std::make_unique<detail::LineTrackingState>(),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
            std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
            std::make_unique<detail::StopState>())};
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

}  // namespace
}  // namespace line_follower
