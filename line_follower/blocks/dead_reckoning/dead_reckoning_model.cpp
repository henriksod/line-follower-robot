// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"

#include <cmath>
#include <iostream>

#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/external/types/rotation.h"

namespace line_follower {

namespace {

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{1e-6};

}  // namespace

Pose const& DeadReckoningModel::getPose() const {
    return pose_;
}

void DeadReckoningModel::setPose(Pose const& new_pose, SystemTime timestamp) {
    pose_ = new_pose;
    time_at_last_step_ = timestamp;
}

double DeadReckoningModel::getForwardVelocity() const {
    return forward_velocity_;
}

double DeadReckoningModel::getAngularVelocity() const {
    return angular_velocity_;
}

void DeadReckoningModel::setEncoderLeftData(EncoderData const& encoder_data_left) {
    left_encoder_data_ = encoder_data_left;
}

void DeadReckoningModel::setEncoderRightData(EncoderData const& encoder_data_right) {
    right_encoder_data_ = encoder_data_right;
}

void DeadReckoningModel::step(SystemTime timestamp) {
    if (time_at_last_step_.system_time_us == 0U) {
        time_at_last_step_ = timestamp;
    }
    auto const time_diff{(timestamp.system_time_us - time_at_last_step_.system_time_us) *
                         kMicrosToSeconds};

    // Calculate wheel velocities
    double left_wheel_velocity = 2.0 * PI * characteristics_.wheel_radius *
                                 left_encoder_data_.revolutions_per_second * time_diff;
    double right_wheel_velocity = 2.0 * PI * characteristics_.wheel_radius *
                                  right_encoder_data_.revolutions_per_second * time_diff;

    // Calculate forward and angular velocities
    forward_velocity_ = (right_wheel_velocity + left_wheel_velocity) / 2.0;
    angular_velocity_ = (right_wheel_velocity - left_wheel_velocity) /
                        (2.0 * characteristics_.distance_between_wheels);

    // Update pose
    pose_.position.x += forward_velocity_ * cos(angular_velocity_);
    pose_.position.y += forward_velocity_ * sin(angular_velocity_);

    geometry::Quaternion<double> rotation{eulerToQuat(EulerRotation{0.0, 0.0, angular_velocity_})};
    pose_.rotation = convert(convert(pose_.rotation) * rotation);

    time_at_last_step_ = timestamp;
}

double DeadReckoningModel::calculateLeftMotorSpeed(double const forward_velocity,
                                                   double const angular_velocity) const {
    return forward_velocity / characteristics_.wheel_radius -
           (characteristics_.distance_between_wheels * angular_velocity) /
               (2.0 * characteristics_.wheel_radius);
}

double DeadReckoningModel::calculateRightMotorSpeed(double const forward_velocity,
                                                    double const angular_velocity) const {
    return forward_velocity / characteristics_.wheel_radius +
           (characteristics_.distance_between_wheels * angular_velocity) /
               (2.0 * characteristics_.wheel_radius);
}

}  // namespace line_follower
