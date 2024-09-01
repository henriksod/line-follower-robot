// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_UTILITIES_TRACK_LOADER_H_
#define LINE_FOLLOWER_BLOCKS_UTILITIES_TRACK_LOADER_H_

#include <string>
#include <vector>

#include "line_follower/external/types/track_lines.h"

namespace line_follower {

namespace track_loader {

/// Load a set of track segments and robot pose from a json file
///
/// @note aborts if the json structure is invalid
///
/// @param[in] path The path to the json file
/// @param[out] track_segments A vector to populate with track segments
/// @param[out] robot_initial_pose The initial pose of the robot in this track
/// @return true if successful, false if failed
bool loadScenarioFromJson(const std::string& path,
                          std::vector<line_follower::TrackSegment>& track_segments,
                          Pose& robot_initial_pose);

}  // namespace track_loader

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_UTILITIES_TRACK_LOADER_H_
