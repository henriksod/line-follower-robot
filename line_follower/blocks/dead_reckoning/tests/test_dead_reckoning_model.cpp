// Copyright (c) 2024 Henrik Söderlund

#include <gtest/gtest.h>

#include <cmath>

#include "line_follower/blocks/common/math.h"
#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
namespace {
constexpr double kRobotWheelRadiusMeters{0.02};
constexpr double kRobotWheelSeparationMeters{0.1};

TEST(DeadReckoningModelTest, TestCorrectPoseAfterUpdate) {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = kRobotWheelRadiusMeters;
    robot_characteristics.distance_between_wheels = kRobotWheelSeparationMeters;

    DeadReckoningModel dead_reckoning_model{robot_characteristics,
                                            Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}};

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.z, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.w, 1.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.z, 0.0, 1e-9);

    EncoderData encoder_data_left{};
    EncoderData encoder_data_right{};

    dead_reckoning_model.step(SystemTime{500000U});

    encoder_data_left.revolutions_per_second = 1.0;
    encoder_data_right.revolutions_per_second = 1.0;
    dead_reckoning_model.setEncoderLeftData(encoder_data_left);
    dead_reckoning_model.setEncoderRightData(encoder_data_right);
    dead_reckoning_model.step(SystemTime{1000000U});

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, PI * kRobotWheelRadiusMeters, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.z, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.w, 1.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.z, 0.0, 1e-9);

    encoder_data_left.revolutions_per_second = -1.0;
    encoder_data_right.revolutions_per_second = -1.0;
    dead_reckoning_model.setEncoderLeftData(encoder_data_left);
    dead_reckoning_model.setEncoderRightData(encoder_data_right);
    dead_reckoning_model.step(SystemTime{1500000U});

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.z, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.w, 1.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.z, 0.0, 1e-9);

    encoder_data_left.revolutions_per_second = 1.0;
    encoder_data_right.revolutions_per_second = -1.0;
    dead_reckoning_model.setEncoderLeftData(encoder_data_left);
    dead_reckoning_model.setEncoderRightData(encoder_data_right);
    dead_reckoning_model.step(SystemTime{2500000U});

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.z, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.w, 0.8090, 1e-4);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.z, -0.5878, 1e-4);

    dead_reckoning_model.setPose(Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, SystemTime{2500000U});

    encoder_data_left.revolutions_per_second = 0.5;
    encoder_data_right.revolutions_per_second = 1.0;
    dead_reckoning_model.setEncoderLeftData(encoder_data_left);
    dead_reckoning_model.setEncoderRightData(encoder_data_right);
    dead_reckoning_model.step(SystemTime{3500000U});

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, 0.0896, 1e-4);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.y, 0.0291, 1e-4);
    EXPECT_NEAR(dead_reckoning_model.getPose().position.z, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.w, 0.9877, 1e-4);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.x, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.y, 0.0, 1e-9);
    EXPECT_NEAR(dead_reckoning_model.getPose().rotation.z, 0.1564, 1e-4);
}

TEST(DeadReckoningModelTest, StepWithNoTimeProgressDoesNotChangePose) {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = kRobotWheelRadiusMeters;
    robot_characteristics.distance_between_wheels = kRobotWheelSeparationMeters;

    DeadReckoningModel dead_reckoning_model{robot_characteristics,
                                            Pose{{1.0, 2.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}};
    dead_reckoning_model.setEncoderLeftData({1.0});
    dead_reckoning_model.setEncoderRightData({1.0});

    SystemTime time{1000000U};
    dead_reckoning_model.step(time);
    auto pose_after_first_step = dead_reckoning_model.getPose();

    // No time progress
    dead_reckoning_model.step(time);
    EXPECT_EQ(dead_reckoning_model.getPose().position.x, pose_after_first_step.position.x);
    EXPECT_EQ(dead_reckoning_model.getPose().position.y, pose_after_first_step.position.y);
}

TEST(DeadReckoningModelTest, MotorSpeedCalculationIsCorrect) {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = kRobotWheelRadiusMeters;
    robot_characteristics.distance_between_wheels = kRobotWheelSeparationMeters;

    DeadReckoningModel dead_reckoning_model{robot_characteristics, Pose{}};
    double fwd_velocity = 1.0;
    double ang_velocity = 0.5;

    double left_speed = dead_reckoning_model.calculateLeftMotorSpeed(fwd_velocity, ang_velocity);
    double right_speed = dead_reckoning_model.calculateRightMotorSpeed(fwd_velocity, ang_velocity);

    double expected_left =
        fwd_velocity / kRobotWheelRadiusMeters -
        (kRobotWheelSeparationMeters * ang_velocity) / (2.0 * kRobotWheelRadiusMeters);
    double expected_right =
        fwd_velocity / kRobotWheelRadiusMeters +
        (kRobotWheelSeparationMeters * ang_velocity) / (2.0 * kRobotWheelRadiusMeters);

    EXPECT_NEAR(left_speed, expected_left, 1e-9);
    EXPECT_NEAR(right_speed, expected_right, 1e-9);
}

TEST(DeadReckoningModelTest, VelocityReportingIsAccurate) {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = kRobotWheelRadiusMeters;
    robot_characteristics.distance_between_wheels = kRobotWheelSeparationMeters;

    DeadReckoningModel dead_reckoning_model{robot_characteristics, Pose{}};
    dead_reckoning_model.setEncoderLeftData({1.0});
    dead_reckoning_model.setEncoderRightData({1.0});
    dead_reckoning_model.step(SystemTime{1000000U});
    dead_reckoning_model.step(SystemTime{2000000U});  // 1s elapsed

    double expected_fwd = 2.0 * PI * kRobotWheelRadiusMeters * 1.0;
    EXPECT_NEAR(dead_reckoning_model.getForwardVelocity(), expected_fwd, 1e-6);
    EXPECT_NEAR(dead_reckoning_model.getAngularVelocity(), 0.0, 1e-6);  // symmetric wheels
}

TEST(DeadReckoningModelTest, StepWithEarlierTimestampIgnored) {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = kRobotWheelRadiusMeters;
    robot_characteristics.distance_between_wheels = kRobotWheelSeparationMeters;

    DeadReckoningModel dead_reckoning_model{robot_characteristics, Pose{}};
    dead_reckoning_model.setEncoderLeftData({1.0});
    dead_reckoning_model.setEncoderRightData({1.0});

    dead_reckoning_model.step(SystemTime{1000000U});
    auto pose_after_first = dead_reckoning_model.getPose();

    dead_reckoning_model.step(SystemTime{900000U});  // earlier
    EXPECT_EQ(dead_reckoning_model.getPose().position.x, pose_after_first.position.x);
    EXPECT_EQ(dead_reckoning_model.getPose().position.y, pose_after_first.position.y);
}
}  // namespace
}  // namespace line_follower
