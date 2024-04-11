// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_ROBOT_CHARACTERISTICS_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_ROBOT_CHARACTERISTICS_H_

namespace line_follower {
/// The characteristics of a differential drive robot
struct DifferentialDriveRobotCharacteristics final {
    /// The wheel radius in meters
    double wheel_radius;

    /// The distance between the wheels in meters
    double distance_between_wheels;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_ROBOT_CHARACTERISTICS_H_
