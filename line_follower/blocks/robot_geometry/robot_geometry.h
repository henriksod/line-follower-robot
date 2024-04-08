// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_ROBOT_GEOMETRY_ROBOT_GEOMETRY_H_
#define LINE_FOLLOWER_BLOCKS_ROBOT_GEOMETRY_ROBOT_GEOMETRY_H_

#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {

namespace robot_geometry {

const geometry::Quaternion<double> kRobotToWorldRotation{0.7071068, 0.0, 0.0, 0.7071068};
const geometry::Quaternion<double> kWorldToRobotRotation{0.7071068, 0.0, 0.0, -0.7071068};
const geometry::Quaternion<double> kIrSensorToRobotRotation{0.7071068, 0.0, 0.0, -0.7071068};
const geometry::Vector3<double> kIrSensorToRobotPosition{0.1, 0.0, 0.0};

}  // namespace robot_geometry

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_ROBOT_GEOMETRY_ROBOT_GEOMETRY_H_
