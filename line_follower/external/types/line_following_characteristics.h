// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_CHARACTERISTICS_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_CHARACTERISTICS_H_

#include "line_follower/external/types/pid_configuration.h"

namespace line_follower {
/// The characteristics of a differential drive robot
struct LineFollowingCharacteristics final {
    /// Steering PID controller parameters
    PIDConfiguration pid_steer_parameters;

    /// Speed PID controller parameters
    PIDConfiguration pid_speed_parameters;

    /// Position state noise
    double position_state_noise;

    /// Position state noise
    double position_derivative_state_noise;

    /// Noise of the ir sensor measurement
    double measurement_noise;

    /// The maximum forward velocity allowed
    double max_forward_velocity;

    /// The ratio of normal speed to apply when turning
    double turning_speed_ratio;

    /// The forward velocity to apply when performing a sharp turn
    double sharp_turn_forward_velocity;

    /// The angular velocity to apply when performing a sharp turn
    double sharp_turn_angular_velocity;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_CHARACTERISTICS_H_
