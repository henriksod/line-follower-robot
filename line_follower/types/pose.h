// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_POSE_H_
#define LINE_FOLLOWER_TYPES_POSE_H_

#include "line_follower/types/position.h"
#include "line_follower/types/rotation.h"

namespace line_follower
{

/// A pose in 3D space, in a coordinate system that
/// follows the right hand rule.
struct Pose final
{
    /// The position of the pose
    Position position;
    /// The rotation of the pose
    QuaternionRotation rotation;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_POSE_H_