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

    // Integral term with optional anti-windup via prospective conditional integration:
    // Compute the tentative full output (P + tentative-I + D) and only accumulate
    // when it stays strictly below _max and at-or-above _min.  The asymmetric bounds
    // are intentional: we stop integrating the moment the output would reach the upper
    // saturation limit (preventing wind-up), but we permit the output to sit exactly at
    // the lower limit so that recovery from positive saturation can integrate freely.
    double const tentative_integral{_integral + error * dt};
    double const tentative_i_out{_Ki * tentative_integral};
    double const tentative_output{p_out + tentative_i_out + d_out};
    if (!_is_first_step &&
        (!_anti_windup || (tentative_output < _max && tentative_output >= _min))) {
        _integral = tentative_integral;
    }
    _is_first_step = false;
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
    _is_first_step = true;
}

}  // namespace line_follower
