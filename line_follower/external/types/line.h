// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_LINE_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_LINE_H_

#include <cstdint>
#include <cstdlib>

#include "line_follower/external/types/position.h"

namespace line_follower {
/// A line with a start position and end position
struct Line final {
    /// The start position of the line.
    Position start;

    /// The end position of the line.
    Position end;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LINE_H_
