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
    Pose result;

    // Rotate the original pose's position, then apply the transform
    Vector3<double> rotated_position = rotated(quaternion, convert(pose.position)) + offset;
    result.position = convert(rotated_position);

    // Compose rotations: new = quaternion * old
    Quaternion<double> combined_rotation = quaternion * convert(pose.rotation);
    result.rotation = convert(combined_rotation);

    return result;
}

}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_ROTATION_UTILS_H_
