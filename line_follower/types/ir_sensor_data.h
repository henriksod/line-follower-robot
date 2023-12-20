// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_IR_SENSOR_DATA_H_
#define LINE_FOLLOWER_TYPES_IR_SENSOR_DATA_H_

#include <cstdlib>
#include <cstdint>
#include <array>

namespace line_follower
{

/// Data of one infrared receiver
struct IrSensorData final
{
    /// Detection of a white surface. True if
    /// the surface was detected as white.
    bool detected_white_surface;

    /// The raw digital reading of the infrared receiver.
    uint16_t digital_reading;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_IR_SENSOR_DATA_H_