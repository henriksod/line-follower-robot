// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_TORQUE_H_
#define LINE_FOLLOWER_TYPES_TORQUE_H_

namespace line_follower {
/// A torque in newtonmillimeters
struct Torque final {
    /// Torque in newtonmillimeters
    double newtonmillimeters;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_TORQUE_H_
