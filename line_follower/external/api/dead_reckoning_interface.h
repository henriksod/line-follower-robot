// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_DEAD_RECKONING_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_DEAD_RECKONING_INTERFACE_H_

#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class DeadReckoningInterface {
 public:
    DeadReckoningInterface() {}

    virtual ~DeadReckoningInterface() noexcept = default;

    DeadReckoningInterface(DeadReckoningInterface const&) = delete;
    DeadReckoningInterface(DeadReckoningInterface&&) = delete;
    DeadReckoningInterface& operator=(DeadReckoningInterface const&) = delete;
    DeadReckoningInterface& operator=(DeadReckoningInterface&&) = delete;

    /// Get the current pose of the model, in robot coordinates
    /// \return The current pose of the model
    virtual Pose const& getPose() const = 0;

    /// Set the current pose of the model, in robot coordinates
    /// @param new_pose The new pose to set
    /// @param timestamp The timestamp of the pose
    virtual void setPose(const Pose& new_pose, SystemTime timestamp) = 0;

    /// Get the forward velocity of the model
    virtual double getForwardVelocity() const = 0;

    /// Get the angular velocity of the model
    virtual double getAngularVelocity() const = 0;

    /// Update the model with new input data for left encoder
    /// @param encoder_data_left The encoder data from the left wheel
    virtual void setEncoderLeftData(const EncoderData& encoder_data_left) = 0;

    /// Update the model with new input data for right encoder
    /// @param encoder_data_right The encoder data from the right wheel
    virtual void setEncoderRightData(const EncoderData& encoder_data_right) = 0;

    /// Calculate the speed of the left motor given forward and angular velocities.
    /// @param forward_velocity The forward velocity of the robot.
    /// @param angular_velocity The angular velocity of the robot.
    /// @return The calculated speed of the left drive motor in rev/s.
    virtual double calculateLeftMotorSpeed(double const forward_velocity,
                                           double const angular_velocity) const = 0;

    /// Calculate the speed of the right motor given forward and angular velocities.
    /// @param forward_velocity The forward velocity of the robot.
    /// @param angular_velocity The angular velocity of the robot.
    /// @return The calculated speed of the right drive motor in rev/s.
    virtual double calculateRightMotorSpeed(double const forward_velocity,
                                            double const angular_velocity) const = 0;

    /// Step the model
    /// @param timestamp The timestamp of the step
    virtual void step(SystemTime timestamp) = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_DEAD_RECKONING_INTERFACE_H_
