// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include <cmath>

#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace geometry {

class Vector3Test : public ::testing::Test {
 protected:
    Vector3<double> v1{1.0, 2.0, 3.0};
    Vector3<double> v2{4.0, 5.0, 6.0};
};

// Construction & Access
TEST_F(Vector3Test, DefaultConstructor) {
    Vector3<double> v;
    EXPECT_EQ(v.x(), 0.0);
    EXPECT_EQ(v.y(), 0.0);
    EXPECT_EQ(v.z(), 0.0);
}

TEST_F(Vector3Test, ParameterizedConstructor) {
    EXPECT_EQ(v1.x(), 1.0);
    EXPECT_EQ(v1.y(), 2.0);
    EXPECT_EQ(v1.z(), 3.0);
}

TEST_F(Vector3Test, CopyConstructor) {
    Vector3<double> v_copy = v1;
    EXPECT_EQ(v_copy, v1);
}

// Set & Zero
TEST_F(Vector3Test, ZeroSetsAllComponentsToZero) {
    v1.zero();
    Vector3<double> expected{0.0, 0.0, 0.0};
    EXPECT_EQ(v1, expected);
}

TEST_F(Vector3Test, SetMethodUpdatesValues) {
    v1.set(7.0, 8.0, 9.0);
    Vector3<double> expected{7.0, 8.0, 9.0};
    EXPECT_EQ(v1, expected);
}

// Arithmetic Operations
TEST_F(Vector3Test, AdditionWithScalar) {
    Vector3<double> result = v1 + 1.0;
    Vector3<double> expected{2.0, 3.0, 4.0};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, SubtractionWithScalar) {
    Vector3<double> result = v1 - 1.0;
    Vector3<double> expected{0.0, 1.0, 2.0};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, MultiplicationWithScalar) {
    Vector3<double> result = v1 * 2.0;
    Vector3<double> expected{2.0, 4.0, 6.0};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, DivisionWithScalar) {
    Vector3<double> result = v1 / 2.0;
    Vector3<double> expected{0.5, 1.0, 1.5};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, VectorAddition) {
    Vector3<double> result = v1 + v2;
    Vector3<double> expected{5.0, 7.0, 9.0};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, VectorSubtraction) {
    Vector3<double> result = v2 - v1;
    Vector3<double> expected{3.0, 3.0, 3.0};
    EXPECT_EQ(result, expected);
}

// Dot & Cross Products
TEST_F(Vector3Test, DotProduct) {
    EXPECT_EQ(v1.dot(v2), 32.0);
}

TEST_F(Vector3Test, CrossProduct) {
    Vector3<double> result = v1.cross(v2);
    Vector3<double> expected{-3.0, 6.0, -3.0};
    EXPECT_EQ(result, expected);
}

// Norm & Normalize
TEST_F(Vector3Test, NormCalculation) {
    EXPECT_NEAR(v1.norm(), std::sqrt(14.0), 1e-9);
}

TEST_F(Vector3Test, NormalizeVector) {
    Vector3<double> v{3.0, 0.0, 4.0};
    v.normalize();
    EXPECT_NEAR(v.norm(), 1.0, 1e-9);
}

// Operators
TEST_F(Vector3Test, UnaryNegation) {
    Vector3<double> result = -v1;
    Vector3<double> expected{-1.0, -2.0, -3.0};
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, ComparisonOperators) {
    Vector3<double> result = v1 < v2;
    Vector3<double> expected{true, true, true};
    EXPECT_EQ(result, expected);

    result = v2 > v1;
    EXPECT_EQ(result, expected);
}

TEST_F(Vector3Test, LogicalOperators) {
    Vector3<double> v_true{1.0, 2.0, 0.0};
    Vector3<double> v_other{0.0, 2.0, 3.0};

    Vector3<double> and_result = v_true && v_other;
    Vector3<double> expected_and{false, true, false};
    EXPECT_EQ(and_result, expected_and);

    Vector3<double> or_result = v_true || v_other;
    Vector3<double> expected_or{true, true, true};
    EXPECT_EQ(or_result, expected_or);
}

// Utilities
TEST_F(Vector3Test, AbsoluteValue) {
    Vector3<double> v{-1.0, 2.0, -3.0};
    Vector3<double> expected{1.0, 2.0, 3.0};
    EXPECT_EQ(v.abs(), expected);
}

TEST_F(Vector3Test, SumOfComponents) {
    EXPECT_EQ(v1.sum(), 6.0);
}

TEST_F(Vector3Test, LerpFunction) {
    Vector3<double> result = lerp(v1, v2, 0.5);
    Vector3<double> expected{2.5, 3.5, 4.5};
    EXPECT_EQ(result, expected);
}

}  // namespace geometry
}  // namespace line_follower
