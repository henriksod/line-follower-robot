// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_PID_PID_H_
#define LINE_FOLLOWER_BLOCKS_PID_PID_H_

#include "line_follower/external/types/pid_configuration.h"

namespace line_follower {

class PID {
 public:
    explicit PID(PIDConfiguration parameters);

    ~PID() noexcept = default;

    // Returns the manipulated variable given a setpoint and current measurement
    double calculate(double setpoint, double measurement, double dt);

    void reset();

 private:
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_PID_PID_H_
