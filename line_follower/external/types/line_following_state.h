// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_LINE_FOLLOWING_STATE_H_
#define LINE_FOLLOWER_TYPES_LINE_FOLLOWING_STATE_H_

#include <array>

#include "line_follower/external/types/covariance.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
/// The kalman filter state of the line following model
struct LineFollowingState final {
    /// Timestamp of the state
    SystemTime timestamp;

    /// Predicted covariance of state
    Covariance2D predicted_covariance;

    /// Predicted line position on ir array
    double predicted_position;

    /// Predicted line position derivative on ir array
    double predicted_position_derivative;

    /// The encoder measurement of the left motor at state
    EncoderData left_encoder_data_input;

    /// The encoder measurement of the right motor at state
    EncoderData right_encoder_data_input;

    /// The output signal to the left motor
    MotorSignal left_motor_signal_output;

    /// The output signal to the right motor
    MotorSignal right_motor_signal_output;

    /// The robot pose at state
    Pose robot_pose;

    /// If the state has been updated once
    bool is_updated_once;

    /// Is valid
    bool valid;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_LINE_FOLLOWING_STATE_H_
