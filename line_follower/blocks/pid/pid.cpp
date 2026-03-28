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
      _integral(0.0),
      _anti_windup(parameters.anti_windup) {}

double PID::calculate(double setpoint, double measurement, double dt) {
    // Calculate error
    double error{setpoint - measurement};

    // Proportional term
    double p_out{_Kp * error};

    // Derivative term
    double derivative{0.0};
    if (dt > 0.0) {
        derivative = (error - _pre_error) / dt;
    }
    double d_out{_Kd * derivative};

    // Integral term with optional anti-windup via conditional integration:
    // Only accumulate when P+D output is not already saturating the output limits.
    double pd_out{p_out + d_out};
    if (!_anti_windup || (pd_out > _min && pd_out < _max)) {
        _integral += error * dt;
    }
    double i_out{_Ki * _integral};

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

void PID::reset() {
    _pre_error = 0.0;
    _integral = 0.0;
}

}  // namespace line_follower
