// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_MOTOR_PIN_CONFIGURATION_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_MOTOR_PIN_CONFIGURATION_H_

#include <cstdint>

#include "line_follower/external/types/pin.h"

namespace line_follower {
/// The pin configuration of a motor
struct MotorPinConfiguration final {
    /// The pin to control the speed of the motor
    AnalogPin pwm_pin;

    /// The in1 used pin to control the direction of the motor
    DigitalPin in1_pin;

    /// The in2 pin to control the direction of the motor
    DigitalPin in2_pin;

    /// Whether to make the robot reverse by default or not
    bool reverse_direction;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_MOTOR_PIN_CONFIGURATION_H_
