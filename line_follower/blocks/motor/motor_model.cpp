// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/blocks/motor/motor_model.h"

#include <algorithm>
#include <cmath>

#include "line_follower/blocks/common/math.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/rotor_speed.h"
#include "line_follower/external/types/torque.h"

namespace line_follower {
Torque MotorModel::getMotorTorque() const {
    return rotor_torque_;
}

RotorSpeed MotorModel::getMotorSpeed() const {
    return target_rotor_speed_;
}

void MotorModel::setMotorSpeed(RotorSpeed const& input) {
    target_rotor_speed_.revolutions_per_second = input.revolutions_per_second;
    rotor_torque_.newtonmillimeters = motor_characteristics_.stall_torque.newtonmillimeters -
                                      fabs(target_rotor_speed_.revolutions_per_second) *
                                          motor_characteristics_.stall_torque.newtonmillimeters /
                                          motor_characteristics_.no_load_speed;
    rotor_torque_.newtonmillimeters *= sgn(target_rotor_speed_.revolutions_per_second);
}
}  // namespace line_follower
