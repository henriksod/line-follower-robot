// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_PIN_CONFIGURATION_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_PIN_CONFIGURATION_H_

#include <cstdint>

#include "line_follower/external/types/pin.h"

namespace line_follower {
/// The pin configuration of an encoder
struct EncoderPinConfiguration final {
    /// The A pin of the encoder
    DigitalPin enc_a_pin;

    /// The B pin of the encoder
    DigitalPin enc_b_pin;

    /// Whether the encoder goes in the reverse direction or not
    bool reverse_direction;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_ENCODER_PIN_CONFIGURATION_H_
