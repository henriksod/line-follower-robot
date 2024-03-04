// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_

#include "line_follower/types/encoder_data.h"
#include "line_follower/types/pose.h"
#include "line_follower/types/robot_characteristics.h"

namespace line_follower {
class DeadReckoningModel final {
 public:
    DeadReckoningModel(DifferentialDriveRobotCharacteristics characteristics, Pose initial_pose)
        : characteristics_{characteristics}, pose_{initial_pose} {}

    ~DeadReckoningModel() noexcept = default;

    DeadReckoningModel(DeadReckoningModel const&) = delete;
    DeadReckoningModel(DeadReckoningModel&&) = delete;
    DeadReckoningModel& operator=(DeadReckoningModel const&) = delete;
    DeadReckoningModel& operator=(DeadReckoningModel&&) = delete;

    /// Get the current pose of the model, in the robot local frame
    Pose const& getPose() const;

    /// Force set the current pose of the model, in the robot local frame
    void setPose(Pose const& new_pose);

    /// Update the model with new input data
    void update(EncoderData const& encoder_data_left, EncoderData const& encoder_data_right,
                double delta_time_seconds);

 private:
    DifferentialDriveRobotCharacteristics characteristics_;
    Pose pose_;
    EncoderData previous_encoder_data_{};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
