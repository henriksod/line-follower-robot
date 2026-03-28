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

    /// Timeout in seconds before transitioning from lost-line recovery to stop
    double lost_line_timeout_seconds{5.0};

    /// Forward velocity when crossing a perpendicular line
    double perpendicular_crossing_forward_velocity{0.2};

    /// Duration in seconds to drive forward when crossing a perpendicular line
    double perpendicular_crossing_duration_seconds{0.3};

    /// EMA filter coefficient for position measurement (1.0 = no filtering, 0.0 = frozen)
    double position_filter_alpha{1.0};

    /// Fraction of LEDs from each side used to detect a sharp turn (default: 1/3)
    double sharp_turn_detection_ratio{1.0 / 3.0};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_CHARACTERISTICS_H_
