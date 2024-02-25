// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_
#define LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_

#include <cstdlib>
#include <cstdint>
#include <array>

namespace line_follower
{

/// The maximum value of a digital reading of an ir sensor
constexpr size_t kDigitalReadingMaxValue{1024U};

/// The maximum number of infrared sensors on an infrared
/// sensor array
constexpr size_t kMaxIrSensorArrayNumberOfLeds{15U};

/// Data of one infrared receiver
struct IrSensorData final
{
    /// Detection of a white surface. True if
    /// the surface was detected as white.
    bool detected_white_surface;

    /// The raw digital reading of the infrared receiver.
    uint16_t digital_reading;
};

/// Data from an infrared sensor array
struct IrSensorArrayData final
{
    /// The readings of each infrared sensor in the
    /// infrared sensor array.
    std::array<IrSensorData, kMaxIrSensorArrayNumberOfLeds> ir_sensor_readings;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_