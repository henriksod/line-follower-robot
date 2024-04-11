// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_TRACK_LINES_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_TRACK_LINES_H_

#include <array>
#include <cstdint>
#include <cstdlib>

#include "line_follower/external/types/line.h"
#include "line_follower/external/types/pose.h"

namespace line_follower {

constexpr size_t kMaxNumberOfLinesPerTrackSegment{50U};

/// A track line segment represented by a pose, length, width, and
/// whiteness intensity value.
struct TrackLineSegment final {
    /// The line of the track segment, with a start and end position.
    /// The line is relative to the pose of the track segment.
    Line line;

    /// The width of the line segment, in millimeters. The width grows
    /// symmetrically on both sides of the line segment centre line.
    double width;

    /// The whiteness intensity of the line segment going from 0 to 1.
    /// 0 is black and 1 is white.
    double whiteness;

    /// Whether the line segment is detectable or not.
    bool visible;
};

/// A segment of a line track. A track segment may consist of a number of
/// track lines.
struct TrackSegment final {
    /// The pose of the track segment. The poses of all track line segments
    /// are relative to this pose.
    Pose pose;

    /// An array of track line segments, which belong to the same
    /// track segment.
    std::array<TrackLineSegment, kMaxNumberOfLinesPerTrackSegment> track_lines;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_TRACK_LINES_H_
