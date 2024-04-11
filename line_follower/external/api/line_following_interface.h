// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_

#include <memory>

#include "line_follower/external/api/dead_reckoning_interface.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class LineFollowingInterface {
 public:
    LineFollowingInterface() {}

    /// Destructor for LineFollowingInterface
    virtual ~LineFollowingInterface() noexcept = default;

    LineFollowingInterface(LineFollowingInterface const&) = delete;
    LineFollowingInterface(LineFollowingInterface&&) = delete;
    LineFollowingInterface& operator=(LineFollowingInterface const&) = delete;
    LineFollowingInterface& operator=(LineFollowingInterface&&) = delete;

    /// Get the predicted line follower state
    virtual LineFollowingState getPredictedState() const = 0;

    /// Set the predicted line follower state
    virtual void setPredictedState(LineFollowingState const& predicted_state) = 0;

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

    /// Predict the next state of the line follower model
    /// @param timestamp The timestamp of the prediction
    virtual void predict(SystemTime timestamp) = 0;

    /// Update the state of the line follower model using ir array data
    /// @param ir_array_data The observed ir sensor array data
    virtual void update(const IrSensorArrayData& ir_array_data) = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_INTERFACE_H_
