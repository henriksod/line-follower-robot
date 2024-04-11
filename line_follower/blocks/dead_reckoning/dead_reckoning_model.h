// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_

#include "line_follower/external/api/dead_reckoning_interface.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

/// Class for modeling the dead reckoning of a differential drive robot
class DeadReckoningModel final : public DeadReckoningInterface {
 public:
    /// Constructor for DeadReckoningModel
    /// @param characteristics The characteristics of the differential drive robot
    /// @param initial_pose The initial pose of the robot
    DeadReckoningModel(DifferentialDriveRobotCharacteristics characteristics, Pose initial_pose)
        : characteristics_{characteristics}, pose_{initial_pose} {}

    /// Destructor for DeadReckoningModel
    ~DeadReckoningModel() noexcept = default;

    // Deleted copy and move constructors and assignment operators
    DeadReckoningModel(const DeadReckoningModel&) = delete;
    DeadReckoningModel(DeadReckoningModel&&) = delete;
    DeadReckoningModel& operator=(const DeadReckoningModel&) = delete;
    DeadReckoningModel& operator=(DeadReckoningModel&&) = delete;

    /// Get the current pose of the model, in robot coordinates
    /// @return The current pose of the model
    Pose const& getPose() const override;

    /// Set the current pose of the model, in robot coordinates
    /// @param new_pose The new pose to set
    /// @param timestamp The timestamp of the pose
    void setPose(const Pose& new_pose, SystemTime timestamp) override;

    /// Get the forward velocity of the model
    double getForwardVelocity() const override;

    /// Get the angular velocity of the model
    double getAngularVelocity() const override;

    /// Update the model with new input data for left encoder
    /// @param encoder_data_left The encoder data from the left wheel
    void setEncoderLeftData(const EncoderData& encoder_data_left) override;

    /// Update the model with new input data for right encoder
    /// @param encoder_data_right The encoder data from the right wheel
    void setEncoderRightData(const EncoderData& encoder_data_right) override;

    /// Calculate the speed of the left motor given forward and angular velocities.
    /// @param forward_velocity The forward velocity of the robot.
    /// @param angular_velocity The angular velocity of the robot.
    /// @return The calculated speed of the left drive motor.
    double calculateLeftMotorSpeed(double const forward_velocity,
                                   double const angular_velocity) const override;

    /// Calculate the speed of the right motor given forward and angular velocities.
    /// @param forward_velocity The forward velocity of the robot.
    /// @param angular_velocity The angular velocity of the robot.
    /// @return The calculated speed of the right drive motor.
    double calculateRightMotorSpeed(double const forward_velocity,
                                    double const angular_velocity) const override;

    /// Step the model
    /// @param timestamp The timestamp of the step
    void step(SystemTime timestamp) override;

 private:
    DifferentialDriveRobotCharacteristics characteristics_;  ///< Characteristics of the robot
    Pose pose_;                                              ///< Current pose of the robot
    EncoderData left_encoder_data_{};
    EncoderData right_encoder_data_{};
    double forward_velocity_{};
    double angular_velocity_{};
    SystemTime time_at_last_step_{};
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_DEAD_RECKONING_DEAD_RECKONING_MODEL_H_
