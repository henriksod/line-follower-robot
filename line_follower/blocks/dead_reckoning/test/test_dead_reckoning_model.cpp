// Copyright (c) 2024 Henrik Söderlund

#include <gtest/gtest.h>

#include <cmath>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/types/encoder_data.h"
#include "line_follower/types/robot_characteristics.h"
#include "line_follower/types/system_time.h"

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

    EXPECT_NEAR(dead_reckoning_model.getPose().position.x, M_PI * kRobotWheelRadiusMeters, 1e-9);
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
}  // namespace
}  // namespace line_follower
