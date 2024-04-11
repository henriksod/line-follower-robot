// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_PID_CONFIGURATION_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_PID_CONFIGURATION_H_

namespace line_follower {
/// Configuration parameters for PID controller
struct PIDConfiguration final {
    /// Proportional gain of controller
    double proportional_gain;
    /// Integral gain of controller
    double integral_gain;
    /// Derivative gain of controller
    double derivative_gain;
    /// Maximum value of output from controller
    double max_value;
    /// Minimum value of output from controller
    double min_value;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_PID_CONFIGURATION_H_
