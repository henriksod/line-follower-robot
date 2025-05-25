// Copyright (c) 2025 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_UTILITIES_CALIBRATION_LOADER_H_
#define LINE_FOLLOWER_BLOCKS_UTILITIES_CALIBRATION_LOADER_H_

#include <string>

#include "line_follower/external/types/line_following_characteristics.h"

namespace line_follower {

namespace calibration_loader {

bool loadCalibrationFromJson(std::string const& calibration_file,
                             LineFollowingCharacteristics& calibration);

}  // namespace calibration_loader

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_UTILITIES_CALIBRATION_LOADER_H_
