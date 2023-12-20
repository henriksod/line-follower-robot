// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_
#define LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_

#include <cstdlib>
#include <cstdint>
#include <array>

#include "line_follower/types/ir_sensor_data.h"

namespace line_follower
{

/// The number of infrared emitter/receiver pairs on the
/// high resolution infrared sensor array
constexpr size_t kHighResIrSensorArrayNumberOfLeds{15U};

/// The horizontal spacing between infrared sensors on the
/// high resolution infrared sensor array. In millimeters.
constexpr double kHighResIrSensorArraySpacing{4.0};

/// The number of infrared emitter/receiver pairs on the
/// low resolution infrared sensor array
constexpr size_t kLowResIrSensorArrayNumberOfLeds{8U};

/// The horizontal spacing between infrared sensors on the
/// low resolution infrared sensor array. In millimeters.
constexpr double kLowResIrSensorArraySpacing{8.0};

/// Data from the high resolution infrared sensor array
struct HighResIrSensorArrayData final
{
    /// The readings of each infrared sensor in the high resolution
    /// infrared sensor array.
    std::array<IrSensorData, kHighResIrSensorArrayNumberOfLeds> ir_sensor_readings;
};

/// Data from the low resolution infrared sensor array
struct LowResIrSensorArrayData final
{
    /// The readings of each infrared sensor in the low resolution
    /// infrared sensor array.
    std::array<IrSensorData, kLowResIrSensorArrayNumberOfLeds> ir_sensor_readings;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_DATA_H_