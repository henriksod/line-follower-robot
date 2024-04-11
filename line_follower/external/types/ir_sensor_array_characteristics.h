// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_CHARACTERISTICS_H_
#define LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_CHARACTERISTICS_H_

#include <cstdlib>

namespace line_follower {
/*
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
   constexpr double kLowResIrSensorArraySpacing{8.0};*/

/// Data from the high resolution infrared sensor array
struct IrSensorArrayCharacteristics final {
    /// The number of infrared emitter/receiver pairs on the
    /// infrared sensor array
    size_t number_of_leds;

    /// The horizontal spacing between infrared sensors on the
    /// infrared sensor array. In millimeters.
    double array_spacing;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_IR_SENSOR_ARRAY_CHARACTERISTICS_H_
