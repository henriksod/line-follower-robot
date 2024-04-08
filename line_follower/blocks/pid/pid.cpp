// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/pid/pid.h"

#include <cmath>

namespace line_follower {

PID::PID(PIDConfiguration parameters)
    : _max(parameters.max_value),
      _min(parameters.min_value),
      _Kp(parameters.proportional_gain),
      _Kd(parameters.derivative_gain),
      _Ki(parameters.integral_gain),
      _pre_error(0.0),
      _integral(0.0) {}

double PID::calculate(double setpoint, double measurement, double dt) {
    // Calculate error
    double error{setpoint - measurement};

    // Proportional term
    double p_out{_Kp * error};

    // Integral term
    _integral += error * _dt;
    double i_out{_Ki * _integral};

    // Derivative term
    double derivative{0.0};
    if (dt > 0.0) {
        /// TODO: Somehow this goes to -nan
        // derivative = (error - _pre_error) / _dt;
    }
    double d_out{_Kd * derivative};

    // Calculate total output
    double output{p_out + i_out + d_out};

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

}  // namespace line_follower
