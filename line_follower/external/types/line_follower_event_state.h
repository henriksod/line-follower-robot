// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWER_EVENT_STATE_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWER_EVENT_STATE_H_

#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
/// The event state of a line follower
struct LineFollowerEventState final {
    /// Timestamp of the state
    SystemTime timestamp;

    /// The encoder measurement of the left motor at state
    EncoderData left_encoder_data_input;

    /// The encoder measurement of the right motor at state
    EncoderData right_encoder_data_input;

    /// The robot pose in world coordinates at state
    Pose global_pose;

    /// The ir array pose in world coordinates at state
    Pose ir_pose;

    /// The statistics of the line follower performance
    LineFollowingStatistics line_following_statistics;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LINE_FOLLOWER_EVENT_STATE_H_
