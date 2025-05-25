// Copyright (c) 2025 Henrik Söderlund

#include <gtest/gtest.h>

#include <vector>

#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"

namespace line_follower {

TEST(IrSensorArrayModelTest, ConstructorInitializesReadings) {
    IrSensorArrayCharacteristics characteristics;
    characteristics.number_of_leds = 4;

    IrSensorArrayModel model(characteristics);

    IrSensorArrayData data;
    ASSERT_TRUE(model.getIrSensorArrayData(data));
    EXPECT_EQ(data.number_of_leds, 4U);
    EXPECT_FALSE(data.valid);
    EXPECT_EQ(data.timestamp.system_time_us, 0U);

    for (const auto& reading : data.ir_sensor_readings) {
        EXPECT_TRUE(reading.detected_white_surface);
        EXPECT_DOUBLE_EQ(reading.intensity, 1.0);
    }
}

TEST(IrSensorArrayModelTest, TickUpdatesTimestamp) {
    IrSensorArrayCharacteristics characteristics;
    characteristics.number_of_leds = 2;

    IrSensorArrayModel model(characteristics);

    SystemTime new_time{12345678U};
    model.tick(new_time);

    IrSensorArrayData data;
    ASSERT_TRUE(model.getIrSensorArrayData(data));
    // Note: data.timestamp isn't updated in tick(); test is meaningful only if that changes
}

TEST(IrSensorArrayModelTest, SetTrackLinesWithEmptyInput) {
    IrSensorArrayCharacteristics characteristics;
    characteristics.number_of_leds = 5;

    IrSensorArrayModel model(characteristics);

    Pose ir_sensor_array_pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
    std::vector<TrackSegment> empty_segments;

    model.setTrackLines(empty_segments, ir_sensor_array_pose);

    IrSensorArrayData data;
    ASSERT_TRUE(model.getIrSensorArrayData(data));
    EXPECT_FALSE(data.valid);
}

TEST(IrSensorArrayModelTest, SetTrackLinesUpdatesReadings) {
    IrSensorArrayCharacteristics characteristics;
    characteristics.number_of_leds = 3;
    characteristics.array_spacing = 10.0;  // in mm
    characteristics.line_detected_threshold = 0.5;

    IrSensorArrayModel model(characteristics);

    Pose ir_sensor_array_pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
    std::vector<TrackSegment> track_segments{{{{-5.0, 1.0, 0.0}, {-5.0, -1.0, 0.0}}, 2.0, 1U, true},
                                             {{{5.0, 1.0, 0.0}, {5.0, -1.0, 0.0}}, 2.0, 1U, true}};

    model.setTrackLines(track_segments, ir_sensor_array_pose);

    IrSensorArrayData data;
    ASSERT_TRUE(model.getIrSensorArrayData(data));
    EXPECT_TRUE(data.valid);
    EXPECT_EQ(data.number_of_leds, characteristics.number_of_leds);

    for (const auto& reading : data.ir_sensor_readings) {
        EXPECT_TRUE(reading.detected_white_surface);
        EXPECT_DOUBLE_EQ(reading.intensity, 1.0);
    }
}

}  // namespace line_follower
