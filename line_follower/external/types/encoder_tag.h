// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_TAG_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_TAG_H_

#include <cstdint>

namespace line_follower {
/// Tag that determines the encoder placement
enum class EncoderTag : uint8_t { kLeft = 0U, kRight, kNone };
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_TAG_H_
