// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/line_following/simple_line_following_model.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "line_follower/blocks/common/maybe.h"
#include "line_follower/blocks/robot_geometry/robot_geometry.h"
#include "line_follower/blocks/state_machine/state_machine.h"
#include "line_follower/external/api/logging.h"

namespace line_follower {

namespace {

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{1e-6};

/// Millimeters to Meters
constexpr double kMillimetersToMeters{0.001};

/// Calculates the average reading position relative to the ir array center.
/// Returns the normalized relative position.
double calculateAverageReadingRelativePosition(IrSensorArrayData const& ir_data) {
    /// TODO: We might have to rethink, maybe we have to go for the most extreme reading...
    size_t number_of_activated_leds{0U};
    IrSensorData const reading{readings[idx]};
    double led_idx{static_cast<double>(idx)};

    // Get the led position relative to the origin, which is the middle of the
    // ir array
    double led_position_rel_origin{led_idx - middle_led_idx};

    // Add to the weighted average reading position
    if (!reading.detected_white_surface) {
        average_reading_position += led_position_rel_origin * (1.0 - reading.intensity);
        ++number_of_activated_leds;
    }
}
if (number_of_activated_leds > 0U) {
    average_reading_position /= static_cast<double>(number_of_activated_leds);
}
<<<<<<< HEAD
average_reading_position /= static_cast<double>(number_of_activated_leds);
return average_reading_position / middle_led_idx;
}

bool isPerpendicularLine(IrSensorArrayData ir_data) {
=======
return average_reading_position / middle_led_idx;
}

void calculateMotorSignals(detail::LineFollowingContext& context, double position,
                           double delta_time_seconds) {
    // We want the center of the robot to be aligned with the line
    double const steer_setpoint{0.0};
    double const out_angular_velocity{
        context.pid_steer.calculate(steer_setpoint, position, delta_time_seconds)};

    // Control the speed for left motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_left_setpoint{context.dead_reckoning_model.calculateLeftMotorSpeed(
        (1.0 - abs(position) * context.characteristics.turning_speed_ratio) *
            context.characteristics.max_forward_velocity,
        out_angular_velocity)};
    double const out_left_wheel_speed{context.pid_left_speed.calculate(
        speed_left_setpoint, context.left_encoder_data.revolutions_per_second, delta_time_seconds)};

    // Control the speed for right motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_right_setpoint{context.dead_reckoning_model.calculateRightMotorSpeed(
        (1.0 - abs(position) * context.characteristics.turning_speed_ratio) *
            context.characteristics.max_forward_velocity,
        out_angular_velocity)};
    double const out_right_wheel_speed{context.pid_right_speed.calculate(
        speed_right_setpoint, context.right_encoder_data.revolutions_per_second,
        delta_time_seconds)};

    context.left_motor_signal.speed.revolutions_per_second = out_left_wheel_speed;
    context.right_motor_signal.speed.revolutions_per_second = out_right_wheel_speed;
}

bool isPerpendicularLine(IrSensorArrayData const& ir_data) {
>>>>>>> 746e44f (Fix line following)
    auto const readings = ir_data.ir_sensor_readings;
    for (size_t idx{0U}; idx < ir_data.number_of_leds; ++idx) {
        if (readings[idx].detected_white_surface) {
            return false;
        }
    }
    return true;
}

<<<<<<< HEAD
=======
bool noLineDetected(IrSensorArrayData const& ir_data) {
    auto const readings = ir_data.ir_sensor_readings;
    for (size_t idx{0U}; idx < ir_data.number_of_leds; ++idx) {
        if (!readings[idx].detected_white_surface) {
            return false;
        }
    }
    return true;
}

bool sharpLeftDetected(IrSensorArrayData const& ir_data) {
    auto const readings = ir_data.ir_sensor_readings;
    for (size_t idx{0U}; idx < ir_data.number_of_leds / 3U; ++idx) {
        if (readings[idx].detected_white_surface) {
            return false;
        }
    }
    for (size_t idx{ir_data.number_of_leds - 1U};
         idx > ir_data.number_of_leds - ir_data.number_of_leds / 3U; --idx) {
        if (!readings[idx].detected_white_surface) {
            return false;
        }
    }
    return true;
}

bool sharpRightDetected(IrSensorArrayData const& ir_data) {
    auto const readings = ir_data.ir_sensor_readings;
    for (size_t idx{0U}; idx < ir_data.number_of_leds / 3U; ++idx) {
        if (!readings[idx].detected_white_surface) {
            return false;
        }
    }
    for (size_t idx{ir_data.number_of_leds - 1U};
         idx > ir_data.number_of_leds - ir_data.number_of_leds / 3U; --idx) {
        if (readings[idx].detected_white_surface) {
            return false;
        }
    }
    return true;
}

bool isTrackingLine(IrSensorArrayData const& ir_data) {
    return !noLineDetected(ir_data) && !isPerpendicularLine(ir_data) &&
           !sharpLeftDetected(ir_data) && !sharpRightDetected(ir_data);
}

>>>>>>> 746e44f (Fix line following)
}  // namespace

Pose const& SimpleLineFollowingModel::getPose() const {
    return dead_reckoning_model_->getPose();
}

void SimpleLineFollowingModel::setPose(const Pose& new_pose, SystemTime timestamp) {
    dead_reckoning_model_->setPose(new_pose, timestamp);
}

MotorSignal SimpleLineFollowingModel::getMotorSignalLeft() const {
<<<<<<< HEAD
    return left_motor_signal_;
}

MotorSignal SimpleLineFollowingModel::getMotorSignalRight() const {
    return right_motor_signal_;
}

void SimpleLineFollowingModel::setEncoderLeftData(EncoderData const& encoder_data_left) {
    left_encoder_data_ = encoder_data_left;
=======
    auto const& context = state_machine_.context();
    return context.left_motor_signal;
}

MotorSignal SimpleLineFollowingModel::getMotorSignalRight() const {
    auto const& context = state_machine_.context();
    return context.right_motor_signal;
}

void SimpleLineFollowingModel::setEncoderLeftData(EncoderData const& encoder_data_left) {
    state_machine_.context().left_encoder_data = encoder_data_left;
>>>>>>> 746e44f (Fix line following)
    dead_reckoning_model_->setEncoderLeftData(encoder_data_left);
}

void SimpleLineFollowingModel::setEncoderRightData(EncoderData const& encoder_data_right) {
<<<<<<< HEAD
    right_encoder_data_ = encoder_data_right;
    dead_reckoning_model_->setEncoderRightData(encoder_data_right);
}

void SimpleLineFollowingModel::calculateMotorSignals(double position, double delta_time_seconds) {
    /// TODO: Maybe we should have robot kinematics specific calculations in dead reckoning model
    /// instead.
    // We want the center of the robot to be aligned with the line
    double const steer_setpoint{0.0};
    double const out_angular_velocity{
        pid_steer_.calculate(steer_setpoint, position, delta_time_seconds)};

    // Control the speed for left motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_left_setpoint{dead_reckoning_model_->calculateLeftMotorSpeed(
        (1.0 - abs(position) * characteristics_.turning_speed_ratio) *
            characteristics_.max_forward_velocity,
        out_angular_velocity)};
    double const out_left_wheel_speed{pid_left_speed_.calculate(
        speed_left_setpoint, left_encoder_data_.revolutions_per_second, delta_time_seconds)};

    // Control the speed for right motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_right_setpoint{dead_reckoning_model_->calculateRightMotorSpeed(
        (1.0 - abs(position) * characteristics_.turning_speed_ratio) *
            characteristics_.max_forward_velocity,
        out_angular_velocity)};
    double const out_right_wheel_speed{pid_right_speed_.calculate(
        speed_right_setpoint, right_encoder_data_.revolutions_per_second, delta_time_seconds)};

    left_motor_signal_.speed.revolutions_per_second = out_left_wheel_speed;
    right_motor_signal_.speed.revolutions_per_second = out_right_wheel_speed;
}

void SimpleLineFollowingModel::update(SystemTime timestamp) {
=======
    state_machine_.context().right_encoder_data = encoder_data_right;
    dead_reckoning_model_->setEncoderRightData(encoder_data_right);
}

void SimpleLineFollowingModel::update(SystemTime timestamp) {
    state_machine_.step();
>>>>>>> 746e44f (Fix line following)
    dead_reckoning_model_->step(timestamp);
}

void SimpleLineFollowingModel::update(const IrSensorArrayData& ir_array_data) {
<<<<<<< HEAD
    if (time_at_last_update_.system_time_us > ir_array_data.timestamp.system_time_us) {
        return;
    }

    if (isPerpendicularLine(ir_array_data)) {
        return;
    }

    if (time_at_last_update_.system_time_us == 0U) {
        time_at_last_update_.system_time_us = ir_array_data.timestamp.system_time_us;
    }
    auto const time_diff{
        (ir_array_data.timestamp.system_time_us - time_at_last_update_.system_time_us) *
        kMicrosToSeconds};

    double const average_steering_error_normalized{
        calculateAverageReadingRelativePosition(ir_array_data)};

    calculateMotorSignals(average_steering_error_normalized, time_diff);
    dead_reckoning_model_->step(ir_array_data.timestamp);

    time_at_last_update_ = ir_array_data.timestamp;
=======
    state_machine_.context().ir_array_data = ir_array_data;
    state_machine_.step();

    dead_reckoning_model_->step(ir_array_data.timestamp);
>>>>>>> 746e44f (Fix line following)
}

SimpleLineFollowingModel::SimpleLineFollowingModel(
    LineFollowingCharacteristics characteristics,
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model)
<<<<<<< HEAD
    : characteristics_{characteristics},
      dead_reckoning_model_{std::move(dead_reckoning_model)},
      pid_left_speed_{characteristics.pid_speed_parameters},
      pid_right_speed_{characteristics.pid_speed_parameters},
      pid_steer_{characteristics.pid_steer_parameters} {
    left_encoder_data_.revolutions_per_second = 0.0;
    right_encoder_data_.revolutions_per_second = 0.0;
}

=======
    : dead_reckoning_model_{std::move(dead_reckoning_model)},
      line_following_context_{detail::LineFollowingContext{
          characteristics, *dead_reckoning_model_, std::make_unique<detail::StartState>(),
          std::make_unique<detail::LineTrackingState>(),
          std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kRight),
          std::make_unique<detail::SharpTurnState>(detail::SharpTurnState::Type::kLeft),
          std::make_unique<detail::StopState>()}},
      state_machine_{line_following_context_.line_tracking_state.get(), line_following_context_} {}

namespace detail {

void StartState::enter(LineFollowingContext& context) {
    LOG_INFO("Entering StartState");
}

void StartState::step(LineFollowingContext& context) {
    // Move forward slowly
    context.left_motor_signal.speed.revolutions_per_second = 1.0;
    context.right_motor_signal.speed.revolutions_per_second = 1.0;
}

Maybe<LineFollowerState*> StartState::transition(LineFollowingContext& context) {
    if (noLineDetected(context.ir_array_data)) {
        return Nothing<LineFollowerState*>();
    }

    if (isPerpendicularLine(context.ir_array_data)) {
        return Nothing<LineFollowerState*>();
    }

    LineFollowerState* new_state = context.line_tracking_state.get();
    return Just(new_state);
}

void SharpTurnState::enter(LineFollowingContext& context) {
    context.time_at_last_update = context.ir_array_data.timestamp;

    LOG_INFO("Entering SharpTurnState");
}

void SharpTurnState::step(LineFollowingContext& context) {
    // double const forward_velocity{context.characteristics.max_forward_velocity *
    //                               context.characteristics.turning_speed_ratio};
    double const forward_velocity{0.1};
    double angular_velocity{0.0};

    switch (type_) {
        case Type::kRight: {
            angular_velocity = -1.0;
            break;
        }
        case Type::kLeft: {
            angular_velocity = 1.0;
            break;
        }
        default: break;
    }

    double const speed_left_setpoint{
        context.dead_reckoning_model.calculateLeftMotorSpeed(forward_velocity, angular_velocity)};

    double const speed_right_setpoint{
        context.dead_reckoning_model.calculateRightMotorSpeed(forward_velocity, angular_velocity)};

    context.left_motor_signal.speed.revolutions_per_second = speed_left_setpoint;
    context.right_motor_signal.speed.revolutions_per_second = speed_right_setpoint;
}

Maybe<LineFollowerState*> SharpTurnState::transition(LineFollowingContext& context) {
    if (isTrackingLine(context.ir_array_data)) {
        LineFollowerState* new_state = context.line_tracking_state.get();
        return Just(new_state);
    }

    /*if (context.ir_array_data.timestamp.system_time_us -
            context.time_at_last_update.system_time_us >
        10.0 * kMicrosToSeconds) {
        LineFollowerState* new_state = context.stop_state.get();
        return Just(new_state);
    }*/

    return Nothing<LineFollowerState*>();
}

void StopState::enter(LineFollowingContext& context) {
    LOG_INFO("Entering StopState");
}

void StopState::step(LineFollowingContext& context) {
    context.left_motor_signal.speed.revolutions_per_second = 0.0;
    context.right_motor_signal.speed.revolutions_per_second = 0.0;
}

Maybe<LineFollowerState*> StopState::transition(LineFollowingContext& context) {
    return Nothing<LineFollowerState*>();
}

void LineTrackingState::enter(LineFollowingContext& context) {
    context.pid_left_speed.reset();
    context.pid_right_speed.reset();
    context.pid_steer.reset();
    context.time_at_last_update = context.ir_array_data.timestamp;

    LOG_INFO("Entering LineTrackingState");
}

void LineTrackingState::step(LineFollowingContext& context) {
    if (context.time_at_last_update.system_time_us >
        context.ir_array_data.timestamp.system_time_us) {
        return;
    }

    if (isPerpendicularLine(context.ir_array_data)) {
        context.time_at_last_update = context.ir_array_data.timestamp;
        return;
    }

    if (noLineDetected(context.ir_array_data)) {
        context.time_at_last_update = context.ir_array_data.timestamp;
        return;
    }

    auto const time_diff{(context.ir_array_data.timestamp.system_time_us -
                          context.time_at_last_update.system_time_us) *
                         kMicrosToSeconds};

    if (time_diff <= 0.0) {
        context.time_at_last_update = context.ir_array_data.timestamp;
        return;
    }

    double const average_steering_error_normalized{
        calculateAverageReadingRelativePosition(context.ir_array_data)};

    calculateMotorSignals(context, average_steering_error_normalized, time_diff);

    context.time_at_last_update = context.ir_array_data.timestamp;
}

Maybe<LineFollowerState*> LineTrackingState::transition(LineFollowingContext& context) {
    if (sharpRightDetected(context.ir_array_data)) {
        LineFollowerState* new_state = context.sharp_turn_right_state.get();
        return Just(new_state);
    }

    if (sharpLeftDetected(context.ir_array_data)) {
        LineFollowerState* new_state = context.sharp_turn_left_state.get();
        return Just(new_state);
    }

    return Nothing<LineFollowerState*>();
}

}  // namespace detail

>>>>>>> 746e44f (Fix line following)
}  // namespace line_follower
