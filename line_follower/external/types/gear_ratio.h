// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_GEAR_RATIO_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_GEAR_RATIO_H_

namespace line_follower {
/// A gear ratio
struct GearRatio final {
    /// The conversion ratio of a gearbox
    double ratio;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_GEAR_RATIO_H_
