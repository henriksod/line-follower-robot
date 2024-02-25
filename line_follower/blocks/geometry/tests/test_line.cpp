// Copyright (c) 2023 Henrik Söderlund

#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/line.h"

namespace line_follower
{
namespace geometry
{
namespace
{

std::vector<std::tuple<Line<double>, Line<double>, bool>> expected_line_intersections = {
    {{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, false},
    {{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}, {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}, true},
    {{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}, {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}, false}
};

TEST(GeometryLineTest, LineIntersection) {
    for (auto& expected_intersections : expected_line_intersections)
    {
        Line<double> line1{std::get<0>(expected_intersections)};
        Line<double> line2{std::get<1>(expected_intersections)};
        EXPECT_EQ(line1.intersectsWith(line2), std::get<2>(expected_intersections));
    }
}

}  // namespace
}  // namespace geometry
}  // namespace line_follower
