// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/line_following/line_following_model.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include "line_follower/blocks/robot_geometry/robot_geometry.h"
#include "line_follower/types/covariance.h"

namespace line_follower {

namespace {

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{1e-6};

/// Millimeters to Meters
constexpr double kMillimetersToMeters{0.001};

/// Default covariance
constexpr Covariance2D kDefaultProcessCovariance{1.0, 0.0, 0.0, 1.0};

/// Calculates the average reading position relative to the ir array center.
/// Returns the normalized relative position.
double calculateAverageReadingRelativePosition(IrSensorArrayData ir_data) {
    /// TODO: We might have to rethink, maybe we have to go for the most extreme reading...
    double average_reading_position{0.0};
    auto const readings = ir_data.ir_sensor_readings;
    double const middle_led_idx{std::floor(ir_data.number_of_leds / 2.0)};
    size_t number_of_activated_leds{0U};
    for (size_t idx{0U}; idx < ir_data.number_of_leds; ++idx) {
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
    average_reading_position /= static_cast<double>(number_of_activated_leds);
    return average_reading_position / middle_led_idx;
}

bool isPerpendicularLine(IrSensorArrayData ir_data) {
    auto const readings = ir_data.ir_sensor_readings;
    for (size_t idx{0U}; idx < ir_data.number_of_leds; ++idx) {
        if (readings[idx].detected_white_surface) {
            return false;
        }
    }
    return true;
}

}  // namespace

Pose const& LineFollowingModel::getPose() const {
    return dead_reckoning_interface_->getPose();
}

void LineFollowingModel::setPose(const Pose& new_pose, SystemTime timestamp) {
    dead_reckoning_interface_->setPose(new_pose, timestamp);
}

LineFollowingState LineFollowingModel::getPredictedState() const {
    return predicted_state_;
}

void LineFollowingModel::setPredictedState(LineFollowingState const& predicted_state) {
    /// TODO: This is ugly...
    predicted_state_ = predicted_state;
    setEncoderLeftData(predicted_state.left_encoder_data_input);
    setEncoderRightData(predicted_state.right_encoder_data_input);
    setPose(predicted_state.robot_pose, predicted_state.timestamp);
    left_motor_signal_ = predicted_state.left_motor_signal_output;
    right_motor_signal_ = predicted_state.right_motor_signal_output;
}

MotorSignal LineFollowingModel::getMotorSignalLeft() const {
    return left_motor_signal_;
}

MotorSignal LineFollowingModel::getMotorSignalRight() const {
    return right_motor_signal_;
}

void LineFollowingModel::setEncoderLeftData(EncoderData const& encoder_data_left) {
    left_encoder_data_ = encoder_data_left;
    dead_reckoning_interface_->setEncoderLeftData(encoder_data_left);
}

void LineFollowingModel::setEncoderRightData(EncoderData const& encoder_data_right) {
    right_encoder_data_ = encoder_data_right;
    dead_reckoning_interface_->setEncoderRightData(encoder_data_right);
}

void LineFollowingModel::calculateMotorSignals(LineFollowingState prediction,
                                               double delta_time_seconds) {
    /// TODO: Maybe we should have robot kinematics specific calculations in dead reckoning model
    /// instead.
    // We want the center of the robot to be aligned with the line
    double const steer_setpoint{0.0};
    double const out_angular_velocity{
        pid_steer_.calculate(steer_setpoint, prediction.predicted_position, delta_time_seconds)};

    // Control the speed for left motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_left_setpoint{dead_reckoning_interface_->calculateLeftMotorSpeed(
        (1.0 - prediction.predicted_position) * characteristics_.max_forward_velocity,
        out_angular_velocity)};
    double const out_left_wheel_speed{pid_left_speed_.calculate(
        speed_left_setpoint, left_encoder_data_.revolutions_per_second, delta_time_seconds)};

    // Control the speed for right motor. We want the robot to go slow in turns and fast on straight
    // paths. The calculation is based on a differential drive robot model.
    double const speed_right_setpoint{dead_reckoning_interface_->calculateRightMotorSpeed(
        (1.0 - prediction.predicted_position) * characteristics_.max_forward_velocity,
        out_angular_velocity)};
    double const out_right_wheel_speed{pid_right_speed_.calculate(
        speed_right_setpoint, right_encoder_data_.revolutions_per_second, delta_time_seconds)};

    left_motor_signal_.speed.revolutions_per_second = out_left_wheel_speed;
    right_motor_signal_.speed.revolutions_per_second = out_right_wheel_speed;

    prediction.left_motor_signal_output = left_motor_signal_;
    prediction.right_motor_signal_output = right_motor_signal_;
}

LineFollowingState LineFollowingModel::preparePredictedState(SystemTime timestamp,
                                                             double delta_time_seconds) const {
    // We have to have updated the state once with an observation in order to start
    // tracking
    if (!predicted_state_.is_updated_once) {
        return predicted_state_;
    }

    LineFollowingState next_predicted_state{predicted_state_};

    double const sensor_to_origin_distance{robot_geometry::kIrSensorToRobotPosition.x()};

    // Calculate predicted state
    next_predicted_state.predicted_position =
        predicted_state_.predicted_position +
        predicted_state_.predicted_position_derivative * delta_time_seconds;

    next_predicted_state.predicted_position_derivative =
        predicted_state_.predicted_position_derivative -
        sensor_to_origin_distance * dead_reckoning_interface_->getAngularVelocity();

    next_predicted_state.left_encoder_data_input = left_encoder_data_;
    next_predicted_state.right_encoder_data_input = right_encoder_data_;
    next_predicted_state.robot_pose = dead_reckoning_interface_->getPose();

    // Calculate predicted state covariance
    double const m00{predicted_state_.predicted_covariance.m00};
    double const m01{predicted_state_.predicted_covariance.m01};
    double const m10{predicted_state_.predicted_covariance.m10};
    double const m11{predicted_state_.predicted_covariance.m11};
    next_predicted_state.predicted_covariance = {
        m00 + m01 * delta_time_seconds + m10 * delta_time_seconds +
            m11 * delta_time_seconds * delta_time_seconds + characteristics_.position_state_noise,
        m01 + m11 * delta_time_seconds, m10 + m11 * delta_time_seconds,
        m11 + characteristics_.position_derivative_state_noise};

    next_predicted_state.valid = true;
    return next_predicted_state;
}

void LineFollowingModel::predict(SystemTime timestamp) {
    if (time_at_last_predict_.system_time_us == 0U) {
        time_at_last_predict_.system_time_us = timestamp.system_time_us;
    }
    auto const time_diff{(timestamp.system_time_us - time_at_last_predict_.system_time_us) *
                         kMicrosToSeconds};

    if (time_diff <= 0.0) {
        return;
    }

    dead_reckoning_interface_->step(timestamp);
    predicted_state_ = preparePredictedState(timestamp, time_diff);
    calculateMotorSignals(predicted_state_, time_diff);

    time_at_last_predict_ = timestamp;
}

void LineFollowingModel::update(const IrSensorArrayData& ir_array_data) {
    if (isPerpendicularLine(ir_array_data)) {
        return;
    }

    predicted_state_.is_updated_once = true;

    double const average_steering_error_normalized{
        calculateAverageReadingRelativePosition(ir_array_data)};

    double const innovation{average_steering_error_normalized -
                            predicted_state_.predicted_position};

    double const m00{predicted_state_.predicted_covariance.m00};
    double const m01{predicted_state_.predicted_covariance.m01};
    double const m10{predicted_state_.predicted_covariance.m10};
    double const m11{predicted_state_.predicted_covariance.m11};

    double const innovation_covariance{m00 + characteristics_.measurement_noise};

    double const kalman_gain_position{m00 / innovation_covariance};
    double const kalman_gain_position_derivative{m10 / innovation_covariance};
    predicted_state_.predicted_position =
        predicted_state_.predicted_position + kalman_gain_position * innovation;

    predicted_state_.predicted_covariance = {(1.0 - kalman_gain_position) * m00, m01,
                                             -kalman_gain_position_derivative * m10, m11};
}

LineFollowingModel::LineFollowingModel(
    LineFollowingCharacteristics characteristics,
    std::unique_ptr<DeadReckoningInterface> dead_reckoning_interface)
    : characteristics_{characteristics},
      dead_reckoning_interface_{std::move(dead_reckoning_interface)},
      predicted_state_{},
      pid_left_speed_{characteristics.pid_speed_parameters},
      pid_right_speed_{characteristics.pid_speed_parameters},
      pid_steer_{characteristics.pid_steer_parameters} {
    predicted_state_.predicted_covariance = kDefaultProcessCovariance;
    left_encoder_data_.revolutions_per_second = 0.0;
    right_encoder_data_.revolutions_per_second = 0.0;
}

}  // namespace line_follower
