// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_

#include "line_follower/types/encoder_data.h"
#include "line_follower/types/pose.h"
#include "line_follower/types/robot_characteristics.h"

namespace line_follower {

/// Class for modeling the dead reckoning of a differential drive robot
class DeadReckoningModel {
 public:
    /// Constructor for DeadReckoningModel
    /// \param characteristics The characteristics of the differential drive robot
    /// \param initial_pose The initial pose of the robot
    DeadReckoningModel(DifferentialDriveRobotCharacteristics characteristics, Pose initial_pose)
        : characteristics_{characteristics}, pose_{initial_pose} {}

    /// Destructor for DeadReckoningModel
    ~DeadReckoningModel() noexcept = default;

    // Deleted copy and move constructors and assignment operators
    DeadReckoningModel(const DeadReckoningModel&) = delete;
    DeadReckoningModel(DeadReckoningModel&&) = delete;
    DeadReckoningModel& operator=(const DeadReckoningModel&) = delete;
    DeadReckoningModel& operator=(DeadReckoningModel&&) = delete;

    /// Get the current pose of the model
    /// \return The current pose of the model
    Pose const& getPose() const;

    /// Set the current pose of the model
    /// \param new_pose The new pose to set
    void setPose(const Pose& new_pose);

    /// Update the model with new input data
    /// \param encoder_data_left The encoder data from the left wheel
    /// \param encoder_data_right The encoder data from the right wheel
    /// \param delta_time_seconds The time elapsed since the last update in seconds
    void update(const EncoderData& encoder_data_left, const EncoderData& encoder_data_right,
                double delta_time_seconds);

 private:
    DifferentialDriveRobotCharacteristics characteristics_;  ///< Characteristics of the robot
    Pose pose_;                                              ///< Current pose of the robot
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
