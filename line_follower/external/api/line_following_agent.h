// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_AGENT_H_

#include <memory>

#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/line_following_interface.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
/// Provides the system time
class LineFollowingAgent final : public IrSensorArrayDataConsumerAgent {
 public:
    LineFollowingAgent(DifferentialDriveRobotCharacteristics robot_characteristics,
                       LineFollowingCharacteristics line_following_characteristics,
                       Pose initial_pose);
    LineFollowingAgent(std::unique_ptr<LineFollowingInterface> line_following_model);
    ~LineFollowingAgent() noexcept;

    LineFollowingAgent(LineFollowingAgent const&) = delete;
    LineFollowingAgent(LineFollowingAgent&&) = delete;
    LineFollowingAgent& operator=(LineFollowingAgent const&) = delete;
    LineFollowingAgent& operator=(LineFollowingAgent&&) = delete;

    /// Get the current pose of the line following model
    /// @return The current pose of the line following model, in world coordinates
    Pose getPose() const;

    /// Get the current pose of the ir sensor array in world coordinates
    /// @return The current pose of the ir sensor array, in world coordinates
    Pose getIrSensorArrayPose() const;

    /// Update the line following model with new input data for left encoder
    /// @param encoder_data_left The encoder data from the left wheel
    void setEncoderLeftData(const EncoderData& encoder_data_left);

    /// Update the line following model with new input data for right encoder
    /// @param encoder_data_right The encoder data from the right wheel
    void setEncoderRightData(const EncoderData& encoder_data_right);

    /// Update the ir sensor array data input of the line following model
    /// @param ir_array_data Infrared sensor array data
    void setIrSensorArrayData(const IrSensorArrayData& ir_array_data);

    /// Get the line following statistics
    LineFollowingStatistics getStatistics();

    /// Attach the left motor to receive signals from the line follower agent
    void attachMotorLeft(MotorSignalConsumerAgent& motor_signal_consumer);

    /// Attach the right motor to receive signals from the line follower agent
    void attachMotorRight(MotorSignalConsumerAgent& motor_signal_consumer);

    /// Step the line following model
    void step();

 private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LINE_FOLLOWING_AGENT_H_
