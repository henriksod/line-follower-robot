// Copyright (c) 2025 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_STATISTICS_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_STATISTICS_H_

#include "line_follower/external/types/system_time.h"

namespace line_follower {
/// The kalman filter state of the line following model
struct LineFollowingStatistics final {
    /// Timestamp of the state
    SystemTime timestamp;

    /// The line follower tracking error
    double tracking_error;

    /// The average speed of the line follower
    double average_speed;

    /// The time spent on the line to follow
    double time_spent_on_line;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWING_STATISTICS_H_
