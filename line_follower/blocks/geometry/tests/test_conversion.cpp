// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/conversion.h"

namespace line_follower {

TEST(ConversionTest, PositionToVector3) {
    Position pos{1.0, 2.0, 3.0};
    auto vec = convert(pos);

    EXPECT_DOUBLE_EQ(vec[0], 1.0);
    EXPECT_DOUBLE_EQ(vec[1], 2.0);
    EXPECT_DOUBLE_EQ(vec[2], 3.0);
}

TEST(ConversionTest, Vector3ToPosition) {
    geometry::Vector3<float> vec{4.0f, 5.0f, 6.0f};
    Position pos = convert(vec);

    EXPECT_FLOAT_EQ(pos.x, 4.0f);
    EXPECT_FLOAT_EQ(pos.y, 5.0f);
    EXPECT_FLOAT_EQ(pos.z, 6.0f);
}

TEST(ConversionTest, EulerRotationToEulerAngles) {
    EulerRotation rot{0.1, 0.2, 0.3};
    auto angles = convert(rot);

    EXPECT_DOUBLE_EQ(angles.roll(), 0.1);
    EXPECT_DOUBLE_EQ(angles.pitch(), 0.2);
    EXPECT_DOUBLE_EQ(angles.yaw(), 0.3);
}

TEST(ConversionTest, ZeroPositionToVector3) {
    Position pos{0.0, 0.0, 0.0};
    auto vec = convert(pos);

    EXPECT_DOUBLE_EQ(vec[0], 0.0);
    EXPECT_DOUBLE_EQ(vec[1], 0.0);
    EXPECT_DOUBLE_EQ(vec[2], 0.0);
}

TEST(ConversionTest, NegativeVector3ToPosition) {
    geometry::Vector3<double> vec{-1.0, -2.0, -3.0};
    Position pos = convert(vec);

    EXPECT_DOUBLE_EQ(pos.x, -1.0);
    EXPECT_DOUBLE_EQ(pos.y, -2.0);
    EXPECT_DOUBLE_EQ(pos.z, -3.0);
}

TEST(ConversionTest, LargeEulerRotationToEulerAngles) {
    EulerRotation rot{1000.0, -1000.0, 500.0};
    auto angles = convert(rot);

    EXPECT_DOUBLE_EQ(angles.roll(), 1000.0);
    EXPECT_DOUBLE_EQ(angles.pitch(), -1000.0);
    EXPECT_DOUBLE_EQ(angles.yaw(), 500.0);
}

TEST(ConversionTest, QuaternionRotationToQuaternion) {
    QuaternionRotation qrot{1.0, 0.0, 0.0, 0.0};
    auto q = convert(qrot);

    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
    EXPECT_DOUBLE_EQ(q.y(), 0.0);
    EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST(ConversionTest, ZeroQuaternionRotationFallback) {
    QuaternionRotation qrot{0.0, 0.0, 0.0, 0.0};
    auto q = convert(qrot);

    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
    EXPECT_DOUBLE_EQ(q.y(), 0.0);
    EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST(ConversionTest, QuaternionToQuaternionRotation) {
    geometry::Quaternion<float> q{0.707f, 0.0f, 0.707f, 0.0f};
    auto qrot = convert(q);

    EXPECT_FLOAT_EQ(qrot.w, 0.707f);
    EXPECT_FLOAT_EQ(qrot.x, 0.0f);
    EXPECT_FLOAT_EQ(qrot.y, 0.707f);
    EXPECT_FLOAT_EQ(qrot.z, 0.0f);
}

TEST(ConversionTest, EulerToQuaternionAndBack) {
    EulerRotation original{0.1, 0.2, 0.3};
    auto maybe_q = eulerToQuat(original);
    ASSERT_TRUE(maybe_q.has_value());
    auto roundtrip = quatToEuler(maybe_q.value());

    EXPECT_NEAR(original.roll, roundtrip.roll, 1e-6);
    EXPECT_NEAR(original.pitch, roundtrip.pitch, 1e-6);
    EXPECT_NEAR(original.yaw, roundtrip.yaw, 1e-6);
}

TEST(ConversionTest, LineToGeometryLineAndBack) {
    Line line{Position{1.0, 2.0, 3.0}, Position{4.0, 5.0, 6.0}};
    auto geom_line = convert(line);
    auto roundtrip = convert(geom_line);

    EXPECT_DOUBLE_EQ(roundtrip.start.x, 1.0);
    EXPECT_DOUBLE_EQ(roundtrip.start.y, 2.0);
    EXPECT_DOUBLE_EQ(roundtrip.start.z, 3.0);
    EXPECT_DOUBLE_EQ(roundtrip.end.x, 4.0);
    EXPECT_DOUBLE_EQ(roundtrip.end.y, 5.0);
    EXPECT_DOUBLE_EQ(roundtrip.end.z, 6.0);
}

}  // namespace line_follower
