// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_PIN_CONFIGURATION_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_PIN_CONFIGURATION_H_

#include <array>
#include <cstdint>

#include "line_follower/external/types/ir_sensor_array_data.h"

namespace line_follower {
/// The pin configuration of an ir sensor array
struct IrSensorArrayPinConfiguration final {
    /// The list of sensor pins
    std::array<uint8_t, kMaxIrSensorArrayNumberOfLeds> sensor_pins;
    /// The list of emitter pins
    std::array<uint8_t, 2U> emitter_pins;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_PIN_CONFIGURATION_H_
