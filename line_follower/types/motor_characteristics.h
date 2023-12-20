// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_
#define LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_

#include <cstdint>

#include "line_follower/types/torque.h"
#include "line_follower/types/gear_ratio.h"

namespace line_follower
{

/// The characteristics of a motor
struct MotorCharacteristics final
{
    /// The maximum revolutions per second the motor can handle
    double max_revolutions_per_second;

    /// The maximum torque the motor can produce
    Torque maximum_torque;

    /// The gear ratio of the motor gearbox
    GearRatio gear_ratio;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_