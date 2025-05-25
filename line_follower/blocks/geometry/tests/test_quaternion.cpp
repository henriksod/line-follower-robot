// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include <iostream>

#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace geometry {

TEST(QuaternionTest, DefaultConstructorIsIdentity) {
    Quaternion<double> q;
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
    EXPECT_DOUBLE_EQ(q.y(), 0.0);
    EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST(QuaternionTest, ConstructorWithValues) {
    Quaternion<float> q(1, 2, 3, 4);
    EXPECT_FLOAT_EQ(q.w(), 1.0f);
    EXPECT_FLOAT_EQ(q.x(), 2.0f);
    EXPECT_FLOAT_EQ(q.y(), 3.0f);
    EXPECT_FLOAT_EQ(q.z(), 4.0f);
}

TEST(QuaternionTest, QuaternionAddition) {
    Quaternion<double> a(1, 2, 3, 4);
    Quaternion<double> b(0.5, -2, 0, 1);
    auto c = a + b;
    EXPECT_DOUBLE_EQ(c.w(), 1.5);
    EXPECT_DOUBLE_EQ(c.x(), 0.0);
    EXPECT_DOUBLE_EQ(c.y(), 3.0);
    EXPECT_DOUBLE_EQ(c.z(), 5.0);
}

TEST(QuaternionTest, QuaternionMultiplication) {
    Quaternion<double> a(1, 0, 1, 0);
    Quaternion<double> b(1, 0.5, 0.5, 0.75);
    auto result = a * b;
    EXPECT_NEAR(result.w(), 0.5, 1e-9);
    EXPECT_NEAR(result.x(), 1.25, 1e-9);
    EXPECT_NEAR(result.y(), 1.5, 1e-9);
    EXPECT_NEAR(result.z(), 0.25, 1e-9);
}

TEST(QuaternionTest, NormSquared) {
    Quaternion<double> q(1, 2, 3, 4);
    EXPECT_DOUBLE_EQ(q.norm_squared(), 1 + 4 + 9 + 16);  // 30
}

TEST(QuaternionTest, IsUnit) {
    auto q = from_euler(EulerAngles<double>{0.1, 0.2, 0.3});
    EXPECT_TRUE(q.value().is_unit(1e-9));
}

TEST(QuaternionTest, Normalize) {
    Quaternion<double> q(0, 3, 0, 4);  // norm = 5
    auto normed = normalize(q);
    ASSERT_TRUE(normed.has_value());
    auto nq = normed.value();
    EXPECT_NEAR(nq.norm(), 1.0, 1e-9);
}

TEST(QuaternionTest, Conjugate) {
    Quaternion<double> q(1, 2, 3, 4);
    auto c = conj(q);
    EXPECT_DOUBLE_EQ(c.w(), 1.0);
    EXPECT_DOUBLE_EQ(c.x(), -2.0);
    EXPECT_DOUBLE_EQ(c.y(), -3.0);
    EXPECT_DOUBLE_EQ(c.z(), -4.0);
}

TEST(QuaternionTest, EulerConversionRoundTrip) {
    EulerAngles<double> e1{0.2, -0.5, 1.0};  // roll, pitch, yaw
    auto q1 = from_euler(e1);
    auto e2 = to_euler(q1.value());
    auto q2 = from_euler(e2);
    EXPECT_TRUE(q1.value().is_unit(1e-9));
    EXPECT_TRUE(q2.value().is_unit(1e-9));
    EXPECT_TRUE(nearly_equal(q1.value(), q2.value(), 1e-3));
}

TEST(QuaternionTest, RotateVector) {
    auto q = from_euler(EulerAngles<double>{0, 0, M_PI / 2});  // 90 deg around Z
    Vector3<double> v{1, 0, 0};
    rotate(q.value(), v);
    EXPECT_NEAR(v.x(), 0.0, 1e-6);
    EXPECT_NEAR(v.y(), 1.0, 1e-6);
    EXPECT_NEAR(v.z(), 0.0, 1e-6);
}

TEST(QuaternionTest, IsZeroDetection) {
    Quaternion<double> q(0, 0, 0, 0);
    EXPECT_TRUE(q.is_zero());
}

TEST(QuaternionTest, ScalarMultiplicationAndDivision) {
    Quaternion<double> q(2, 4, 6, 8);
    q *= 0.5;
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 2.0);
    EXPECT_DOUBLE_EQ(q.y(), 3.0);
    EXPECT_DOUBLE_EQ(q.z(), 4.0);
    q /= 2;
    EXPECT_DOUBLE_EQ(q.w(), 0.5);
}

TEST(QuaternionTest, DotProduct) {
    Quaternion<double> a(1, 2, 3, 4);
    Quaternion<double> b(2, 3, 4, 5);
    double d = dot(a, b);
    EXPECT_DOUBLE_EQ(d, 1 * 2 + 2 * 3 + 3 * 4 + 4 * 5);  // = 40
}

TEST(QuaternionTest, CrossProduct) {
    Quaternion<double> a(0, 1, 0, 0);
    Quaternion<double> b(0, 0, 1, 0);
    auto c = cross(a, b);
    EXPECT_DOUBLE_EQ(c.x(), 0);
    EXPECT_DOUBLE_EQ(c.y(), 0);
    EXPECT_DOUBLE_EQ(c.z(), 1);
}

}  // namespace geometry
}  // namespace line_follower
