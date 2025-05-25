// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/geometry/utils/line_utils.h"

namespace line_follower {

TEST(LineUtilsTest, SweepAlongWidthCount) {
    geometry::Line<double> base_line{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    double width = 2.0;
    std::size_t resolution = 10;

    auto lines = geometry::sweepAlongWidth(base_line, width, resolution);
    EXPECT_EQ(lines.size(), resolution);
}

TEST(LineUtilsTest, SweepAlongWidthSymmetry) {
    geometry::Line<double> base_line{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    double width = 2.0;
    std::size_t resolution = 3;

    auto lines = geometry::sweepAlongWidth(base_line, width, resolution);
    ASSERT_EQ(lines.size(), resolution);

    // Compute the expected Y-offsets from left to right (symmetric around Y = 0)
    std::vector<double> expected_offsets{-1.0, 0.0, 1.0};

    for (std::size_t i = 0; i < resolution; ++i) {
        double actual_y = lines[i].from()[1];
        EXPECT_NEAR(actual_y, expected_offsets[i], 1e-6) << "Line " << i << " Y-offset mismatch";
    }
}

TEST(LineUtilsTest, SweepAlongWidthZeroWidth) {
    geometry::Line<double> base_line{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    auto lines = geometry::sweepAlongWidth(base_line, 0.0, 5);

    for (const auto& line : lines) {
        EXPECT_NEAR(line.from()[1], 0.0, 1e-6);
        EXPECT_NEAR(line.to()[1], 0.0, 1e-6);
    }
}

}  // namespace line_follower
