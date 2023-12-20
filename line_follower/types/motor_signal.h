// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_MOTOR_SIGNAL_H_
#define LINE_FOLLOWER_TYPES_MOTOR_SIGNAL_H_

#include "line_follower/types/rotor_speed.h"

namespace line_follower
{

/// A signal to a motor
struct MotorSignal final
{
    /// Rotor speed in revolutions per second
    RotorSpeed speed;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_MOTOR_SIGNAL_H_