// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/blocks/motor/motor_model.h"

#include <math.h>
#include <algorithm>

#include "line_follower/types/torque.h"
#include "line_follower/types/rotor_speed.h"
#include "line_follower/types/motor_characteristics.h"

namespace line_follower
{

void MotorModel::tick()
{
    double rotor_speed_error{target_rotor_speed_.revolutions_per_second
                             - rotor_speed_.revolutions_per_second};
    double power_on_motor_shaft{2.0 * M_PI
                                * motor_characteristics_.maximum_torque.newtonmeters
                                * motor_characteristics_.gear_ratio.ratio
                                * rotor_speed_error};
    rotor_speed_.revolutions_per_second = std::min(
        motor_characteristics_.max_revolutions_per_second,
        power_on_motor_shaft / (2.0 * M_PI * motor_characteristics_.maximum_torque.newtonmeters));
}

void MotorModel::setMotorSpeed(RotorSpeed const& input)
{
    target_rotor_speed_.revolutions_per_second = input.revolutions_per_second;
}

}  // namespace line_follower
