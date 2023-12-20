// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_ROTOR_SPEED_H_
#define LINE_FOLLOWER_TYPES_ROTOR_SPEED_H_

namespace line_follower
{

/// A rotor speed
struct RotorSpeed final
{
    /// Rotor speed in revolutions per second
    double revolutions_per_second;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_ROTOR_SPEED_H_