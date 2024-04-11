// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_ENCODER_DATA_H_
#define LINE_FOLLOWER_TYPES_ENCODER_DATA_H_

#include <cstdint>

namespace line_follower {
/// Data from a rotary encoder sensor
struct EncoderData final {
    /// A calculated value of the revolutions per second
    /// sensed by the rotary encoder. A positive value indicates
    /// rotation in the clockwise direction. A negative value indicates
    /// rotation in the anti-clockwise direction.
    double revolutions_per_second;

    /// The step position of the rotary encoder. Rotation in the clockwise
    /// direction increments the step position, and rotation in the
    // anti-clockwise
    /// direction decrements the step position.
    int64_t step_position;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_ENCODER_DATA_H_
