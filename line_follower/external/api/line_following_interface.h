// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_

#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class LineFollowingInterface {
 public:
    LineFollowingInterface() {}

    virtual ~LineFollowingInterface() noexcept = default;

    LineFollowingInterface(const LineFollowingInterface&) = delete;
    LineFollowingInterface(LineFollowingInterface&&) = delete;
    LineFollowingInterface& operator=(const LineFollowingInterface&) = delete;
    LineFollowingInterface& operator=(LineFollowingInterface&&) = delete;

    /// Get the current pose of the model, in robot coordinates
    /// @return The current pose of the model
    virtual Pose const& getPose() const = 0;

    /// Set the current pose of the model, in robot coordinates
    /// @param new_pose The new pose to set
    /// @param timestamp The timestamp of the pose
    virtual void setPose(const Pose& new_pose, SystemTime timestamp) = 0;

    /// Get the desired motor signal for the left motor
    virtual MotorSignal getMotorSignalLeft() const = 0;

    /// Get the desired motor signal for the left motor
    virtual MotorSignal getMotorSignalRight() const = 0;

    /// Update the model with new input data for left encoder
    /// @param encoder_data_left The encoder data from the left wheel
    virtual void setEncoderLeftData(const EncoderData& encoder_data_left) = 0;

    /// Update the model with new input data for right encoder
    /// @param encoder_data_right The encoder data from the right wheel
    virtual void setEncoderRightData(const EncoderData& encoder_data_right) = 0;

    /// Get the line following statistics
    virtual LineFollowingStatistics getStatistics() = 0;

    /// Update the line follower model using timestamp
    /// @param timestamp The current timestamp
    virtual void update(SystemTime timestamp) = 0;

    /// Update the line follower model using ir array data
    /// @param ir_array_data The observed ir sensor array data
    virtual void update(const IrSensorArrayData& ir_array_data) = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_
