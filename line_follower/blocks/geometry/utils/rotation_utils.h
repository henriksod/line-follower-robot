// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_ROTATION_UTILS_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_ROTATION_UTILS_H_

#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/vector.h"
#include "line_follower/external/types/pose.h"

namespace line_follower {
namespace geometry {
namespace detail {
/// A 90 degree rotation around z
const geometry::Quaternion<double> k90DegreesAroundZ{0.7071068, 0.0, 0.0, 0.7071068};
}  // namespace detail

inline Pose transformedPose(Quaternion<double> const& quaternion, Vector3<double> const& offset,
                            Pose const& pose) {
    Pose new_pose{pose};
    new_pose.position = convert(rotated(quaternion, convert(new_pose.position)) + offset);
    new_pose.rotation = convert(convert(new_pose.rotation) * quaternion);
    return new_pose;
}

inline Pose transformedPose(Quaternion<double> const& quaternion, Pose const& pose) {
    Vector3<double> zero_offset{};
    return transformedPose(quaternion, zero_offset, pose);
}

}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_ROTATION_UTILS_H_
