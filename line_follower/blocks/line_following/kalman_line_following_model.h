// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_KALMAN_LINE_FOLLOWING_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_KALMAN_LINE_FOLLOWING_MODEL_H_

#include <memory>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/pid/pid.h"
#include "line_follower/external/api/line_following_interface.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

/// Class for modeling line following of a line using a ir array sensor
class LineFollowingModel : public LineFollowingInterface {
 public:
    /// Constructor for LineFollowingModel
    /// @param characteristics The characteristics of the line following algorithm
    /// @param dead_reckoning_model Interface to a dead reckoning model
    LineFollowingModel(LineFollowingCharacteristics characteristics,
                       std::unique_ptr<DeadReckoningModel> dead_reckoning_model);

    /// Destructor for LineFollowingModel
    ~LineFollowingModel() noexcept final = default;

    // Deleted copy and move constructors and assignment operators
    LineFollowingModel(const LineFollowingModel&) = delete;
    LineFollowingModel(LineFollowingModel&&) = delete;
    LineFollowingModel& operator=(const LineFollowingModel&) = delete;
    LineFollowingModel& operator=(LineFollowingModel&&) = delete;

    /// Get the predicted line follower state
    LineFollowingState getPredictedState() const;

    /// Set the predicted line follower state
    void setPredictedState(LineFollowingState const& predicted_state);

    /// Get the predicted line follower state
    /// @param timestamp The timestamp of the prediction
    /// @param delta_time_seconds The delta time between predicted states
    LineFollowingState preparePredictedState(SystemTime timestamp, double delta_time_seconds) const;

    /// Get the current pose of the model, in robot coordinates
    /// @return The current pose of the model
    Pose const& getPose() const override;

    /// Set the current pose of the model, in robot coordinates
    /// @param new_pose The new pose to set
    /// @param timestamp The timestamp of the pose
    void setPose(const Pose& new_pose, SystemTime timestamp) override;

    /// Get the desired motor signal for the left motor
    MotorSignal getMotorSignalLeft() const override;

    /// Get the desired motor signal for the left motor
    MotorSignal getMotorSignalRight() const override;

    /// Update the model with new input data for left encoder
    /// @param encoder_data_left The encoder data from the left wheel
    void setEncoderLeftData(const EncoderData& encoder_data_left) override;

    /// Update the model with new input data for right encoder
    /// @param encoder_data_right The encoder data from the right wheel
    void setEncoderRightData(const EncoderData& encoder_data_right) override;

    /// Get the line following statistics
    LineFollowingStatistics getStatistics();

    /// Update the line follower model using timestamp
    /// @param timestamp The current timestamp
    void update(SystemTime timestamp) override;

    /// Update the line follower model using ir array data
    /// @param ir_array_data The observed ir sensor array data
    void update(const IrSensorArrayData& ir_array_data) override;

 private:
    LineFollowingCharacteristics characteristics_;
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model_;
    LineFollowingState predicted_state_;
    PID pid_left_speed_;
    PID pid_right_speed_;
    PID pid_steer_;
    MotorSignal left_motor_signal_{};
    MotorSignal right_motor_signal_{};
    EncoderData left_encoder_data_{};
    EncoderData right_encoder_data_{};
    SystemTime time_at_last_predict_{};

    /// Calculate the desired motor signals based on predicted state
    /// of the ir array sensor
    void calculateMotorSignals(LineFollowingState prediction, double delta_time_seconds);
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_LINE_FOLLOWING_MODEL_H_
