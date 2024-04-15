// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_MOTOR_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_MOTOR_INTERFACE_H_

#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {
class MotorInterface {
 public:
    MotorInterface() {}

    virtual ~MotorInterface() noexcept = default;

    MotorInterface(MotorInterface const&) = delete;
    MotorInterface(MotorInterface&&) = delete;
    MotorInterface& operator=(MotorInterface const&) = delete;
    MotorInterface& operator=(MotorInterface&&) = delete;

    virtual void setMotorSpeed(RotorSpeed const& input) = 0;
    virtual void initialize() = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_MOTOR_INTERFACE_H_
