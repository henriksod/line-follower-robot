// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/line.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace geometry {

class LineTest : public ::testing::Test {
 protected:
    using Vec = Vector3<double>;
    using L = Line<double>;

    Vec origin{0.0, 0.0, 0.0};
    Vec x_axis{1.0, 0.0, 0.0};
    Vec y_axis{0.0, 1.0, 0.0};
    Vec z_axis{0.0, 0.0, 1.0};
};

TEST_F(LineTest, DefaultConstructorInitializesToZero) {
    L line;
    EXPECT_EQ(line.from(), Vec(0.0, 0.0, 0.0));
    EXPECT_EQ(line.to(), Vec(0.0, 0.0, 0.0));
}

TEST_F(LineTest, FromToConstructorSetsEndpoints) {
    L line(origin, x_axis);
    EXPECT_EQ(line.from(), origin);
    EXPECT_EQ(line.to(), x_axis);
}

TEST_F(LineTest, SingleVectorConstructorInitializesFromOrigin) {
    L line(x_axis);
    EXPECT_EQ(line.from(), origin);
    EXPECT_EQ(line.to(), x_axis);
}

TEST_F(LineTest, CenterComputesCorrectMidpoint) {
    L line(origin, Vec(2.0, 2.0, 2.0));
    EXPECT_EQ(line.center(), Vec(1.0, 1.0, 1.0));
}

TEST_F(LineTest, SetUpdatesEndpoints) {
    L line;
    line.set(x_axis, y_axis);
    EXPECT_EQ(line.from(), x_axis);
    EXPECT_EQ(line.to(), y_axis);
}

TEST_F(LineTest, EqualityOperatorWorks) {
    L a(origin, x_axis);
    L b(origin, x_axis);
    L c(origin, y_axis);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TEST_F(LineTest, InequalityOperatorWorks) {
    L a(origin, x_axis);
    L b(origin, y_axis);
    EXPECT_TRUE(a != b);
}

TEST_F(LineTest, NegationReversesDirection) {
    L a(origin, x_axis);
    L b = -a;
    EXPECT_EQ(b.from(), x_axis);
    EXPECT_EQ(b.to(), origin);
}

TEST_F(LineTest, IntersectsWithReturnsTrueForCrossingLines) {
    L a(Vec(0, 0, 0), Vec(1, 1, 0));
    L b(Vec(0, 1, 0), Vec(1, 0, 0));
    EXPECT_TRUE(a.intersectsWith(b));
}

TEST_F(LineTest, IntersectsWithReturnsFalseForNonIntersectingLines) {
    L a(Vec(0, 0, 0), Vec(1, 0, 0));
    L b(Vec(0, 1, 0), Vec(1, 1, 0));
    EXPECT_FALSE(a.intersectsWith(b));
}

TEST_F(LineTest, IntersectsWithAnyDetectsIntersection) {
    L a(Vec(0, 0, 0), Vec(1, 1, 0));
    std::vector<L> lines = {L(Vec(0, 1, 0), Vec(1, 0, 0)), L(Vec(2, 2, 0), Vec(3, 3, 0))};
    EXPECT_TRUE(a.intersectsWithAny(lines));
}

TEST_F(LineTest, IntersectsWithAnyReturnsFalseIfNoIntersection) {
    L a(Vec(0, 0, 0), Vec(1, 1, 0));
    std::vector<L> lines = {L(Vec(2, 2, 0), Vec(3, 3, 0)), L(Vec(4, 4, 0), Vec(5, 5, 0))};
    EXPECT_FALSE(a.intersectsWithAny(lines));
}

TEST_F(LineTest, IntersectsWithAllReturnsTrueIfAllIntersect) {
    L a(Vec(0, 0, 0), Vec(1, 1, 0));
    std::vector<L> lines = {L(Vec(0, 1, 0), Vec(1, 0, 0)), L(Vec(1, 0, 0), Vec(0, 1, 0))};
    EXPECT_TRUE(a.intersectsWithAll(lines));
}

TEST_F(LineTest, IntersectsWithAllReturnsFalseIfAnyMisses) {
    L a(Vec(0, 0, 0), Vec(1, 1, 0));
    std::vector<L> lines = {L(Vec(0, 1, 0), Vec(1, 0, 0)), L(Vec(3, 3, 0), Vec(4, 4, 0))};
    EXPECT_FALSE(a.intersectsWithAll(lines));
}

}  // namespace geometry
}  // namespace line_follower
