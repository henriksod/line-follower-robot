// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_

#include "line_follower/types/pose.h"
#include "line_follower/types/track_lines.h"
#include "line_follower/types/ir_sensor_array_data.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_interface.h"

namespace line_follower
{

static_assert(kMaximumTrackLineWhiteness == kDigitalReadingMaxValue,
              "Digital reading and maximum track line whiteness do not match!");

class IrSensorArrayModel final : public IrSensorArrayInterface
{

void resetIrSensorArrayData(IrSensorArrayData& ir_sensor_array_data)
{
    for (auto& ir_sensor_data : ir_sensor_array_data.ir_sensor_readings)
    {
        ir_sensor_data.detected_white_surface = true;
        ir_sensor_data.digital_reading = kDigitalReadingMaxValue;
    }
}

 public:
    explicit IrSensorArrayModel(IrSensorArrayCharacteristics ir_array_characteristics)
        : ir_array_characteristics_{ir_array_characteristics},
          ir_sensor_array_data_{},
          current_track_segment_{}
    {
      resetIrSensorArrayData(ir_sensor_array_data_);
    }

    ~IrSensorArrayModel() noexcept final = default;

    IrSensorArrayModel(IrSensorArrayModel const&)            = delete;
    IrSensorArrayModel(IrSensorArrayModel&&)                 = delete;
    IrSensorArrayModel& operator=(IrSensorArrayModel const&) = delete;
    IrSensorArrayModel& operator=(IrSensorArrayModel&&)      = delete;

    void tick() override;
    bool getIrSensorArrayData(IrSensorArrayData& output) const override;

    /// @brief Set the reference track lines to convert into ir sensor readings.
    /// @param track_segment The track segment which the track lines are part of.
    /// @param ir_sensor_array_pose_in_track_segment The pose of the ir sensor array
    ///                                              relative tothe track segment pose.
    void setTrackLines(TrackSegment const& track_segment,
                       Pose const& ir_sensor_array_pose_in_track_segment);

 private:
    IrSensorArrayCharacteristics const ir_array_characteristics_;
    IrSensorArrayData ir_sensor_array_data_;
    TrackSegment current_track_segment_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_