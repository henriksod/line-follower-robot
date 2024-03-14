// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_LINE_UTILS_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_LINE_UTILS_H_

#include <algorithm>
#include <cstdlib>
#include <vector>

#include "line_follower/blocks/geometry/line.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace geometry {

/// @brief Sweep over the width of a line
/// @param line The line
/// @param width The width of the line to sweep over
/// @param resolution Optionally set the resolution of the sweep (number of
// lines)
/// @returns A vector of lines along the width of the line
template <class T>
inline std::vector<Line<T> > sweepAlongWidth(Line<T> const& line, double width,
                                             std::size_t resolution = 50U) {
    std::vector<Line<T> > lines{};

    Vector3<double> line_vector{normalized(line.to() - line.from())};
    auto orthogonal_vector{rotated(detail::k90DegreesAroundZ, line_vector)};

    Line<double> left_line{line.from() + orthogonal_vector * width / 2.0,
                           line.to() + orthogonal_vector * width / 2.0};
    Line<double> right_line{line.from() - orthogonal_vector * width / 2.0,
                            line.to() - orthogonal_vector * width / 2.0};

    double lerp_value{0.0};

    for (std::size_t idx{0U}; idx < resolution; ++idx) {
        lines.push_back(Line<double>(lerp(left_line.from(), right_line.from(), lerp_value),
                                     lerp(left_line.to(), right_line.to(), lerp_value)));

        lerp_value = std::min(1.0, lerp_value + 1.0 / static_cast<double>(resolution));
    }

    return lines;
}
}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_UTILS_LINE_UTILS_H_
