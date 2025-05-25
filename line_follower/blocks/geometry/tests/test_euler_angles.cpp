// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/euler_angles.h"

namespace line_follower {
namespace geometry {

TEST(EulerAnglesTest, DefaultConstructor) {
    EulerAngles<double> e;
    EXPECT_DOUBLE_EQ(e.roll(), 0.0);
    EXPECT_DOUBLE_EQ(e.pitch(), 0.0);
    EXPECT_DOUBLE_EQ(e.yaw(), 0.0);
    EXPECT_TRUE(e.is_zero());
    EXPECT_FALSE(e.is_nan());
    EXPECT_FALSE(e.is_inf());
    EXPECT_TRUE(e.is_finite());
}

TEST(EulerAnglesTest, ParameterizedConstructor) {
    EulerAngles<float> e(1.1f, 2.2f, 3.3f);
    EXPECT_FLOAT_EQ(e.roll(), 1.1f);
    EXPECT_FLOAT_EQ(e.pitch(), 2.2f);
    EXPECT_FLOAT_EQ(e.yaw(), 3.3f);
    EXPECT_FALSE(e.is_zero(1e-6));
}

TEST(EulerAnglesTest, CopyConstructorAndAssignment) {
    EulerAngles<double> e1(0.5, -0.5, 1.0);
    EulerAngles<float> e2{e1};
    EXPECT_NEAR(e2.roll(), 0.5, 1e-6);
    EXPECT_NEAR(e2.pitch(), -0.5, 1e-6);
    EXPECT_NEAR(e2.yaw(), 1.0, 1e-6);

    EulerAngles<double> e3;
    e3 = e2;  // assignment operator
    EXPECT_DOUBLE_EQ(e3.roll(), e2.roll());
    EXPECT_DOUBLE_EQ(e3.pitch(), e2.pitch());
    EXPECT_DOUBLE_EQ(e3.yaw(), e2.yaw());
}

TEST(EulerAnglesTest, ArithmeticOperators) {
    EulerAngles<double> e1(1.0, 2.0, 3.0);
    EulerAngles<double> e2(0.5, 1.0, 1.5);

    e1 += 1.0;
    EXPECT_DOUBLE_EQ(e1.roll(), 2.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 3.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 4.0);

    e1 -= 1.0;
    EXPECT_DOUBLE_EQ(e1.roll(), 1.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 3.0);

    e1 *= 2.0;
    EXPECT_DOUBLE_EQ(e1.roll(), 2.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 4.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 6.0);

    e1 /= 2.0;
    EXPECT_DOUBLE_EQ(e1.roll(), 1.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 3.0);

    e1 += e2;
    EXPECT_DOUBLE_EQ(e1.roll(), 1.5);
    EXPECT_DOUBLE_EQ(e1.pitch(), 3.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 4.5);

    e1 -= e2;
    EXPECT_DOUBLE_EQ(e1.roll(), 1.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 3.0);

    e1 *= e2;
    EXPECT_DOUBLE_EQ(e1.roll(), 0.5);
    EXPECT_DOUBLE_EQ(e1.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 4.5);

    e1 /= e2;
    EXPECT_DOUBLE_EQ(e1.roll(), 1.0);
    EXPECT_DOUBLE_EQ(e1.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(e1.yaw(), 3.0);
}

TEST(EulerAnglesTest, NegationOperator) {
    EulerAngles<double> e(1.0, -2.0, 3.0);
    EulerAngles<double> neg_e = -e;
    EXPECT_DOUBLE_EQ(neg_e.roll(), -1.0);
    EXPECT_DOUBLE_EQ(neg_e.pitch(), 2.0);
    EXPECT_DOUBLE_EQ(neg_e.yaw(), -3.0);
}

TEST(EulerAnglesTest, IsZeroWithEpsilon) {
    EulerAngles<double> e(1e-8, -1e-9, 0.0);
    EXPECT_FALSE(e.is_zero());
    EXPECT_TRUE(e.is_zero(1e-7));
}

}  // namespace geometry
}  // namespace line_follower
