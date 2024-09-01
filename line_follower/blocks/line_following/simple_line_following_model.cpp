// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/line_following/simple_line_following_model.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "line_follower/blocks/robot_geometry/robot_geometry.h"

namespace line_follower {

namespace {

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{1e-6};

/// Millimeters to Meters
constexpr double kMillimetersToMeters{0.001};

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
    if (number_of_activated_leds > 0U) {
        average_reading_position /= static_cast<double>(number_of_activated_leds);
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

Pose const& SimpleLineFollowingModel::getPose() const {
    return dead_reckoning_model_->getPose();
}

void SimpleLineFollowingModel::setPose(const Pose& new_pose, SystemTime timestamp) {
    dead_reckoning_model_->setPose(new_pose, timestamp);
}

MotorSignal SimpleLineFollowingModel::getMotorSignalLeft() const {
    return left_motor_signal_;
}

MotorSignal SimpleLineFollowingModel::getMotorSignalRight() const {
    return right_motor_signal_;
}

void SimpleLineFollowingModel::setEncoderLeftData(EncoderData const& encoder_data_left) {
    left_encoder_data_ = encoder_data_left;
    dead_reckoning_model_->setEncoderLeftData(encoder_data_left);
}

void SimpleLineFollowingModel::setEncoderRightData(EncoderData const& encoder_data_right) {
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
    dead_reckoning_model_->step(timestamp);
}

void SimpleLineFollowingModel::update(const IrSensorArrayData& ir_array_data) {
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
}

SimpleLineFollowingModel::SimpleLineFollowingModel(
    LineFollowingCharacteristics characteristics,
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model)
    : characteristics_{characteristics},
      dead_reckoning_model_{std::move(dead_reckoning_model)},
      pid_left_speed_{characteristics.pid_speed_parameters},
      pid_right_speed_{characteristics.pid_speed_parameters},
      pid_steer_{characteristics.pid_steer_parameters} {
    left_encoder_data_.revolutions_per_second = 0.0;
    right_encoder_data_.revolutions_per_second = 0.0;
}

}  // namespace line_follower
