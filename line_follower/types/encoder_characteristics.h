// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_ENCODER_CHARACTERISTICS_H_
#define LINE_FOLLOWER_TYPES_ENCODER_CHARACTERISTICS_H_

#include <cstdint>

namespace line_follower {
/// The characteristics of an encoder
struct EncoderCharacteristics final {
  /// The number of encoder counts for one
  /// revolution of the rotor shaft
  uint16_t counts_per_revolution;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_ENCODER_CHARACTERISTICS_H_
