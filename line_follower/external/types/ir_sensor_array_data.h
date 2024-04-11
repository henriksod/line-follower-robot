// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_DATA_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_DATA_H_

#include <array>
#include <cstdint>
#include <cstdlib>

#include "line_follower/external/types/system_time.h"

namespace line_follower {
/// The maximum value of a digital reading of an ir sensor
constexpr size_t kDigitalReadingMaxValue{1024U};

/// The maximum number of infrared sensors on an infrared
/// sensor array
constexpr size_t kMaxIrSensorArrayNumberOfLeds{15U};

/// Data of one infrared receiver
struct IrSensorData final {
    /// Detection of a white surface. True if
    /// the surface was detected as white.
    bool detected_white_surface;

    /// The converted reading intensity level of the ir sensor.
    /// Value goes from 0 to 1 where 0 is black and 1 is white.
    double intensity;
};

/// Data from an infrared sensor array
struct IrSensorArrayData final {
    /// Is data valid
    bool valid;
    /// The timestamp when reading the data
    SystemTime timestamp;
    /// The number of leds in this data, up to a maximum of
    /// kMaxIrSensorArrayNumberOfLeds.
    size_t number_of_leds;
    /// The readings of each infrared sensor in the
    /// infrared sensor array.
    std::array<IrSensorData, kMaxIrSensorArrayNumberOfLeds> ir_sensor_readings;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_IR_SENSOR_ARRAY_DATA_H_
