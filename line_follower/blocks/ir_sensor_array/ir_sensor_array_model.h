// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_

#include <vector>

#include "line_follower/external/api/ir_sensor_array_interface.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"
#include "line_follower/external/types/track_lines.h"

namespace line_follower {

class IrSensorArrayModel final : public IrSensorArrayInterface {
    void resetIrSensorArrayData(IrSensorArrayData& ir_sensor_array_data) {
        ir_sensor_array_data.valid = false;
        for (auto& ir_sensor_data : ir_sensor_array_data.ir_sensor_readings) {
            ir_sensor_data.detected_white_surface = true;
            ir_sensor_data.intensity = 1.0;
        }
    }

 public:
    explicit IrSensorArrayModel(IrSensorArrayCharacteristics ir_array_characteristics)
        : ir_array_characteristics_{ir_array_characteristics}, ir_sensor_array_data_{} {
        resetIrSensorArrayData(ir_sensor_array_data_);
    }

    ~IrSensorArrayModel() noexcept final = default;

    IrSensorArrayModel(IrSensorArrayModel const&) = delete;
    IrSensorArrayModel(IrSensorArrayModel&&) = delete;
    IrSensorArrayModel& operator=(IrSensorArrayModel const&) = delete;
    IrSensorArrayModel& operator=(IrSensorArrayModel&&) = delete;

    void tick(SystemTime const timestamp) override;
    bool getIrSensorArrayData(IrSensorArrayData& output) const override;
    void initialize() override {};
    void calibrate(size_t const iterations) override { static_cast<void>(iterations); };

    /// @brief Set the reference track lines to convert into ir sensor readings.
    /// @param track_segments The track segments of the track to detect lines in.
    /// @param ir_sensor_array_pose The pose of the ir sensor
    /// array relative to world coordinates.
    void setTrackLines(std::vector<TrackSegment> const& track_segments,
                       Pose const& ir_sensor_array_pose);

 private:
    IrSensorArrayCharacteristics const ir_array_characteristics_;
    IrSensorArrayData ir_sensor_array_data_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_MODEL_H_
