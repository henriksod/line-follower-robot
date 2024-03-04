// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"

#include <math.h>

#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/types/rotation.h"

namespace line_follower {

Pose const& DeadReckoningModel::getPose() const {
    return pose_;
}

void DeadReckoningModel::setPose(Pose const& new_pose) {
    pose_ = new_pose;
}

void DeadReckoningModel::update(EncoderData const& encoder_data_left,
                                EncoderData const& encoder_data_right, double delta_time_seconds) {
    /// TODO: We could apply slippage based on friction constant of wheels here
    double left_wheel_velocity{2.0 * M_PI * characteristics_.wheel_radius *
                               encoder_data_left.revolutions_per_second * delta_time_seconds};
    double right_wheel_velocity{2.0 * M_PI * characteristics_.wheel_radius *
                                encoder_data_right.revolutions_per_second * delta_time_seconds};

    double forward_velocity{(right_wheel_velocity + left_wheel_velocity) / 2.0};
    double angular_velocity{(right_wheel_velocity - left_wheel_velocity) /
                            (2.0 * characteristics_.distance_between_wheels)};

    pose_.position.x += forward_velocity * cos(angular_velocity);
    pose_.position.y += forward_velocity * sin(angular_velocity);

    geometry::Quaternion<double> rotation{eulerToQuat(EulerRotation{0.0, 0.0, angular_velocity})};
    pose_.rotation = convert(convert(pose_.rotation) * rotation);
}

}  // namespace line_follower
