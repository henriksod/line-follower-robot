// Copyright (c) 2024 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/line_following/line_following_model.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
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

    line_following_characteristics.pid_speed_parameters.proportional_gain = 1.0;
    line_following_characteristics.pid_speed_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.min_value = 0.0;
    line_following_characteristics.pid_speed_parameters.max_value = 2.0;

    line_following_characteristics.pid_steer_parameters.proportional_gain = 1.0;
    line_following_characteristics.pid_steer_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.min_value = -2.0;
    line_following_characteristics.pid_steer_parameters.max_value = 2.0;

    line_following_characteristics.max_forward_velocity = 1.0;
    line_following_characteristics.measurement_noise = 0.0;

    return line_following_characteristics;
}

TEST(LineFollowingModelTest, TestPredictNoInput) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};

    LineFollowingModel line_following_model{createLineFollowingCharacteristics(),
                                            std::move(dead_reckoning_model)};

    line_following_model.update(SystemTime{0UL});

    EXPECT_NEAR(line_following_model.getMotorSignalLeft().speed.revolutions_per_second, 0.0, 1e-6);
    EXPECT_NEAR(line_following_model.getMotorSignalRight().speed.revolutions_per_second, 0.0, 1e-6);
}

TEST(LineFollowingModelTest, TestPredictInitialState) {
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model{std::make_unique<DeadReckoningModel>(
        createRobotCharacteristics(), Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}})};

    LineFollowingModel line_following_model{createLineFollowingCharacteristics(),
                                            std::move(dead_reckoning_model)};

    IrSensorArrayData first_observation{};
    resetIrSensorArrayData(first_observation);
    first_observation.valid = true;
    first_observation.timestamp = SystemTime{5000UL};
    first_observation.number_of_leds = 15U;
    first_observation.ir_sensor_readings.at(2U).detected_white_surface = false;
    first_observation.ir_sensor_readings.at(2U).intensity = 0.0;

    EncoderData encoder_data{};
    encoder_data.revolutions_per_second = 0.0;
    line_following_model.setEncoderLeftData(encoder_data);
    line_following_model.setEncoderRightData(encoder_data);

    line_following_model.update(first_observation.timestamp);

    EXPECT_NEAR(line_following_model.getMotorSignalLeft().speed.revolutions_per_second, 0.0, 1e-6);
    EXPECT_NEAR(line_following_model.getMotorSignalRight().speed.revolutions_per_second, 0.0, 1e-6);

    line_following_model.update(first_observation);

    line_following_model.update(SystemTime{10000UL});

    EXPECT_GT(line_following_model.getMotorSignalLeft().speed.revolutions_per_second, 0.0);
    EXPECT_GT(line_following_model.getMotorSignalRight().speed.revolutions_per_second, 0.0);

    /// TODO: Extend the test
}
}  // namespace
}  // namespace line_follower
