// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/external/types/pose.h"

namespace line_follower {

TEST(RotationUtilsTest, TransformedPoseIdentity) {
    Pose pose{Position{1.0, 2.0, 3.0}, QuaternionRotation{1.0, 0.0, 0.0, 0.0}};
    geometry::Quaternion<double> identity{1.0, 0.0, 0.0, 0.0};
    geometry::Vector3<double> offset{0.0, 0.0, 0.0};

    auto result = geometry::transformedPose(identity, offset, pose);

    EXPECT_DOUBLE_EQ(result.position.x, 1.0);
    EXPECT_DOUBLE_EQ(result.position.y, 2.0);
    EXPECT_DOUBLE_EQ(result.position.z, 3.0);
    EXPECT_DOUBLE_EQ(result.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(result.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(result.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(result.rotation.z, 0.0);
}

TEST(RotationUtilsTest, TransformedPoseWithOffset) {
    Pose pose{Position{0.0, 0.0, 0.0}, QuaternionRotation{1.0, 0.0, 0.0, 0.0}};
    geometry::Quaternion<double> identity{1.0, 0.0, 0.0, 0.0};
    geometry::Vector3<double> offset{1.0, 2.0, 3.0};

    auto result = geometry::transformedPose(identity, offset, pose);

    EXPECT_DOUBLE_EQ(result.position.x, 1.0);
    EXPECT_DOUBLE_EQ(result.position.y, 2.0);
    EXPECT_DOUBLE_EQ(result.position.z, 3.0);
}

TEST(RotationUtilsTest, TransformedPoseWithRotationOnly) {
    Pose pose{Position{1.0, 0.0, 0.0}, QuaternionRotation{1.0, 0.0, 0.0, 0.0}};
    geometry::Quaternion<double> rot90z{0.7071068, 0.0, 0.0, 0.7071068};  // 90 deg around Z
    geometry::Vector3<double> offset{0.0, 0.0, 0.0};

    auto result = geometry::transformedPose(rot90z, offset, pose);

    EXPECT_NEAR(result.position.x, 0.0, 1e-5);
    EXPECT_NEAR(result.position.y, 1.0, 1e-5);
    EXPECT_NEAR(result.position.z, 0.0, 1e-5);
}

TEST(RotationUtilsTest, TransformedPoseWithRotationAndOffset) {
    Pose pose{Position{1.0, 0.0, 0.0}, QuaternionRotation{1.0, 0.0, 0.0, 0.0}};
    geometry::Quaternion<double> rot90z{0.7071068, 0.0, 0.0, 0.7071068};
    geometry::Vector3<double> offset{0.0, 2.0, 0.0};

    auto result = geometry::transformedPose(rot90z, offset, pose);

    EXPECT_NEAR(result.position.x, 0.0, 1e-5);
    EXPECT_NEAR(result.position.y, 3.0, 1e-5);
    EXPECT_NEAR(result.position.z, 0.0, 1e-5);
}

}  // namespace line_follower
