// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_ROTATION_H_
#define LINE_FOLLOWER_TYPES_ROTATION_H_

namespace line_follower {
/// A rotation represented by a quaternion.
/// Rotation follows the right hand rule.
struct QuaternionRotation final {
  /// Real component of the quaternion
  double w;

  /// Imaginary x component of the quaternion
  double x;

  /// Imaginary y component of the quaternion
  double y;

  /// Imaginary z component of the quaternion
  double z;
};

/// A rotation represented by euler angles.
/// Rotation follows the right hand rule.
struct EulerRotation final {
  /// Rotation around x, in radians
  double roll;

  /// Rotation around y, in radians
  double pitch;

  /// Rotation around z, in radians
  double yaw;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_ROTATION_H_
