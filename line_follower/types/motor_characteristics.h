// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_
#define LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_

#include <cstdint>

#include "line_follower/types/gear_ratio.h"
#include "line_follower/types/torque.h"

namespace line_follower {
/// The characteristics of a motor
struct MotorCharacteristics final {
    /// The maximum speed of the motor at no load,
    /// in revolutions per second
    double no_load_speed;

    /// The stall torque of the motor, in newton millimeters
    Torque stall_torque;

    /// The gear ratio of the motor gearbox, where a value under
    /// one is down gearing, and a value above one is up gearing
    GearRatio gear_ratio;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_MOTOR_CHARACTERISTICS_H_
