// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_PIN_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_PIN_H_

#include <cstdint>
#include <cstdlib>

#include "line_follower/external/types/position.h"

namespace line_follower {
/// The state of a pin
enum class PinState : uint32_t {
    /// The pin is in low voltage state
    kLow = 0U,
    /// The pin is in high voltage state
    kHigh
};

/// The mode of a pin
enum class PinMode : uint32_t {
    /// Pin is in input mode
    kInput = 0U,
    /// Pin is in output mode
    kOutput,
    /// Pin is in input mode with pullup to VCC
    kInputPullup
};

/// The information of a digital pin of a microcontroller
struct DigitalPin final {
    /// The id of the pin
    uint32_t id;
    /// The state of the pin
    PinState state;
    /// The mode of the pin
    PinMode mode;
};

/// The information of an analog pin of a microcontroller
struct AnalogPin final {
    /// The id of the pin
    uint32_t id;
    /// The value of the pin
    uint32_t value;
    /// The mode of the pin
    PinMode mode;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_PIN_H_
