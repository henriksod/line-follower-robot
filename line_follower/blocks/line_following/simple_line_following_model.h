// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_SIMPLE_LINE_FOLLOWING_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_SIMPLE_LINE_FOLLOWING_MODEL_H_

<<<<<<< HEAD
#include <memory>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/pid/pid.h"
=======
#include <cstdint>
#include <memory>
#include <utility>

#include "line_follower/blocks/common/maybe.h"
#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/pid/pid.h"
#include "line_follower/blocks/state_machine/state_machine.h"
>>>>>>> 746e44f (Fix line following)
#include "line_follower/external/api/line_following_interface.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

<<<<<<< HEAD
=======
namespace detail {

class StartState;
class LineTrackingState;
class SharpTurnState;
class StopState;

/// @brief  A context used for line following state machine
struct LineFollowingContext {
    LineFollowingCharacteristics characteristics;
    DeadReckoningModel const& dead_reckoning_model;
    IrSensorArrayData ir_array_data;
    PID pid_left_speed;
    PID pid_right_speed;
    PID pid_steer;
    MotorSignal left_motor_signal;
    MotorSignal right_motor_signal;
    EncoderData left_encoder_data;
    EncoderData right_encoder_data;
    SystemTime time_at_last_update;

    std::unique_ptr<detail::StartState> start_state;
    std::unique_ptr<detail::LineTrackingState> line_tracking_state;
    std::unique_ptr<detail::SharpTurnState> sharp_turn_right_state;
    std::unique_ptr<detail::SharpTurnState> sharp_turn_left_state;
    std::unique_ptr<detail::StopState> stop_state;

    LineFollowingContext(LineFollowingCharacteristics characteristics_,
                         DeadReckoningModel const& dead_reckoning_model_,
                         std::unique_ptr<detail::StartState> start_state_,
                         std::unique_ptr<detail::LineTrackingState> line_tracking_state_,
                         std::unique_ptr<detail::SharpTurnState> sharp_turn_right_state_,
                         std::unique_ptr<detail::SharpTurnState> sharp_turn_left_state_,
                         std::unique_ptr<detail::StopState> stop_state_)
        : characteristics{characteristics_},
          dead_reckoning_model{dead_reckoning_model_},
          ir_array_data{},
          pid_left_speed{characteristics.pid_speed_parameters},
          pid_right_speed{characteristics.pid_speed_parameters},
          pid_steer{characteristics.pid_steer_parameters},
          left_motor_signal{},
          right_motor_signal{},
          left_encoder_data{},
          right_encoder_data{},
          time_at_last_update{},
          start_state{std::move(start_state_)},
          line_tracking_state{std::move(line_tracking_state_)},
          sharp_turn_right_state{std::move(sharp_turn_right_state_)},
          sharp_turn_left_state{std::move(sharp_turn_left_state_)},
          stop_state{std::move(stop_state_)} {}
};

using LineFollowerState = State<LineFollowingContext>;

/// @brief Start state
class StartState : public LineFollowerState {
 public:
    StartState() {}

    ~StartState() noexcept = default;

    StartState(StartState const&) = delete;
    StartState(StartState&&) = delete;
    StartState& operator=(StartState const&) = delete;
    StartState& operator=(StartState&&) = delete;

    /// @brief Enter function
    ///
    /// @param[in, out] context A state machine context
    void enter(LineFollowingContext& context);

    /// @brief State step function
    ///
    /// @param[in, out] context A state machine context
    void step(LineFollowingContext& context);

    /// @brief State transition function
    ///
    /// @param[in, out] context A state machine context
    /// @returns A new state or noop
    Maybe<LineFollowerState*> transition(LineFollowingContext& context);
};

/// @brief Line tracking state
class LineTrackingState : public LineFollowerState {
 public:
    LineTrackingState() {}

    ~LineTrackingState() noexcept = default;

    LineTrackingState(LineTrackingState const&) = delete;
    LineTrackingState(LineTrackingState&&) = delete;
    LineTrackingState& operator=(LineTrackingState const&) = delete;
    LineTrackingState& operator=(LineTrackingState&&) = delete;

    /// @brief Enter function
    ///
    /// @param[in, out] context A state machine context
    void enter(LineFollowingContext& context);

    /// @brief State step function
    ///
    /// @param[in] context A state machine context
    void step(LineFollowingContext& context);

    /// @brief State transition function
    ///
    /// @param[in] context A state machine context
    /// @returns A new state or noop
    Maybe<LineFollowerState*> transition(LineFollowingContext& context);
};

/// @brief Sharp turn state
class SharpTurnState : public LineFollowerState {
 public:
    enum class Type : std::uint8_t { kNone = 0U, kRight, kLeft };

    explicit SharpTurnState(Type type) : type_{type} {}

    ~SharpTurnState() noexcept = default;

    SharpTurnState(SharpTurnState const&) = delete;
    SharpTurnState(SharpTurnState&&) = delete;
    SharpTurnState& operator=(SharpTurnState const&) = delete;
    SharpTurnState& operator=(SharpTurnState&&) = delete;

    /// @brief Enter function
    ///
    /// @param[in, out] context A state machine context
    void enter(LineFollowingContext& context);

    /// @brief State step function
    ///
    /// @param[in] context A state machine context
    void step(LineFollowingContext& context);

    /// @brief State transition function
    ///
    /// @param[in] context A state machine context
    /// @returns A new state or noop
    Maybe<LineFollowerState*> transition(LineFollowingContext& context);

 private:
    Type type_;
};

/// @brief Stop state
class StopState : public LineFollowerState {
 public:
    StopState() {}

    ~StopState() noexcept = default;

    StopState(StopState const&) = delete;
    StopState(StopState&&) = delete;
    StopState& operator=(StopState const&) = delete;
    StopState& operator=(StopState&&) = delete;

    /// @brief Enter function
    ///
    /// @param[in, out] context A state machine context
    void enter(LineFollowingContext& context);

    /// @brief State step function
    ///
    /// @param[in] context A state machine context
    void step(LineFollowingContext& context);

    /// @brief State transition function
    ///
    /// @param[in] context A state machine context
    /// @returns A new state or noop
    Maybe<LineFollowerState*> transition(LineFollowingContext& context);
};

}  // namespace detail

>>>>>>> 746e44f (Fix line following)
/// Class for modeling line following of a line using a ir array sensor
class SimpleLineFollowingModel : public LineFollowingInterface {
 public:
    /// Constructor for SimpleLineFollowingModel
    /// @param characteristics The characteristics of the line following algorithm
    /// @param dead_reckoning_model Interface to a dead reckoning model
    SimpleLineFollowingModel(LineFollowingCharacteristics characteristics,
                             std::unique_ptr<DeadReckoningModel> dead_reckoning_model);

    /// Destructor for SimpleLineFollowingModel
    ~SimpleLineFollowingModel() noexcept final = default;

    // Deleted copy and move constructors and assignment operators
    SimpleLineFollowingModel(const SimpleLineFollowingModel&) = delete;
    SimpleLineFollowingModel(SimpleLineFollowingModel&&) = delete;
    SimpleLineFollowingModel& operator=(const SimpleLineFollowingModel&) = delete;
    SimpleLineFollowingModel& operator=(SimpleLineFollowingModel&&) = delete;

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

    /// Update the line follower model using timestamp
    /// @param timestamp The current timestamp
    void update(SystemTime timestamp) override;

    /// Update the line follower model using ir array data
    /// @param ir_array_data The observed ir sensor array data
    void update(const IrSensorArrayData& ir_array_data) override;

 private:
<<<<<<< HEAD
    LineFollowingCharacteristics characteristics_;
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model_;
    PID pid_left_speed_;
    PID pid_right_speed_;
    PID pid_steer_;
    MotorSignal left_motor_signal_{};
    MotorSignal right_motor_signal_{};
    EncoderData left_encoder_data_{};
    EncoderData right_encoder_data_{};
    SystemTime time_at_last_update_{};
=======
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model_;
    detail::LineFollowingContext line_following_context_;
    StateMachine<detail::LineFollowingContext> state_machine_;
>>>>>>> 746e44f (Fix line following)

    /// Calculate the desired motor signals based on predicted state
    /// of the ir array sensor
    void calculateMotorSignals(double position, double delta_time_seconds);
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_LINE_FOLLOWING_SIMPLE_LINE_FOLLOWING_MODEL_H_
