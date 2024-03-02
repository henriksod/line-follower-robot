// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_POSITION_H_
#define LINE_FOLLOWER_TYPES_POSITION_H_

namespace line_follower {
/// A position in 3D space.
/// Coordinate system follows the right hand rule.
struct Position final {
    /// Position along x, in meters
    double x;

    /// Position along y, in meters
    double y;

    /// Position along z, in meters
    double z;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_POSITION_H_
