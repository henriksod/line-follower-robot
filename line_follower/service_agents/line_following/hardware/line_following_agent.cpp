// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/external/api/line_following_agent.h"

#include <array>
#include <cstdint>
#include <memory>
#include <utility>

#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/geometry/vector.h"
#include "line_follower/blocks/line_following/line_following_model.h"
#include "line_follower/blocks/robot_geometry/robot_geometry.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

namespace {
// Kalman filter state history size
constexpr size_t kStateHistorySize{5U};
}  // namespace

/// TODO: Line following logic
/// Kalman filter to track the line position on the 1 dimensional ir sensor array
/// PID that follows the predicted position of the line based on the kalman filter
/// Special case when either all leds detect line on one side or all leds in total detect a line
/// When all leds detect a line, we continue forward and do not use the observation
/// When all leds on one side are detected, we make the pid follow an orthogonal line to the one
/// before and we predict the line to be detected on the outermost edge. If no observation of a
/// line, we continue predicting and follow the predicted path
class LineFollowingAgent::Impl final {
 public:
    Impl(DifferentialDriveRobotCharacteristics robot_characteristics,
         LineFollowingCharacteristics line_following_characteristics, Pose initial_pose)
        : line_following_model_{std::make_unique<LineFollowingModel>(
              line_following_characteristics,
              std::make_unique<DeadReckoningModel>(
                  robot_characteristics,
                  geometry::transformedPose(robot_geometry::kWorldToRobotRotation, initial_pose)))},
          left_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          right_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          time_agent_{},
          time_at_last_step_{time_agent_.getSystemTime()} {
        LOG_INFO("Created line following agent (hardware)", "");
    }

    explicit Impl(std::unique_ptr<LineFollowingInterface> line_following_model)
        : line_following_model_{std::move(line_following_model)},
          left_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          right_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          time_agent_{},
          time_at_last_step_{time_agent_.getSystemTime()} {
        LOG_INFO("Created line following agent (hardware)", "");
    }

    Pose getPose() const {
        return geometry::transformedPose(robot_geometry::kRobotToWorldRotation,
                                         line_following_model_->getPose());
    }

    Pose getIrSensorArrayPose() const {
        return geometry::transformedPose(
            robot_geometry::kRobotToWorldRotation,
            geometry::transformedPose(convert(line_following_model_->getPose().rotation),
                                      convert(line_following_model_->getPose().position),
                                      {convert(robot_geometry::kIrSensorToRobotPosition),
                                       convert(robot_geometry::kIrSensorToRobotRotation)}));
    }

    void setEncoderLeftData(const EncoderData& encoder_data_left) {
        line_following_model_->setEncoderLeftData(encoder_data_left);
    }

    void setEncoderRightData(const EncoderData& encoder_data_right) {
        line_following_model_->setEncoderRightData(encoder_data_right);
    }

    void setIrSensorArrayData(const IrSensorArrayData& ir_array_data) {
        new_ir_array_data_ = ir_array_data;
    }

    void attachMotorLeft(MotorSignalConsumerAgent& motor_signal_consumer) {
        motor_signal_consumer.attach(*left_motor_signal_producer_);
    }

    void attachMotorRight(MotorSignalConsumerAgent& motor_signal_consumer) {
        motor_signal_consumer.attach(*right_motor_signal_producer_);
    }

    void step() {
        SystemTime current_time{time_agent_.getSystemTime()};

        if (new_ir_array_data_.valid) {
            line_following_model_->predict(new_ir_array_data_.timestamp);
            line_following_model_->update(new_ir_array_data_);
            new_ir_array_data_ = {};
        }

        line_following_model_->predict(current_time);

        left_motor_signal_producer_->sendData(line_following_model_->getMotorSignalLeft());
        right_motor_signal_producer_->sendData(line_following_model_->getMotorSignalRight());
    }

 private:
    std::unique_ptr<LineFollowingInterface> line_following_model_;
    std::unique_ptr<MotorSignalProducerAgent> left_motor_signal_producer_;
    std::unique_ptr<MotorSignalProducerAgent> right_motor_signal_producer_;
    TimeAgent time_agent_;
    SystemTime time_at_last_step_;
    IrSensorArrayData new_ir_array_data_{};
    std::array<LineFollowingState, kStateHistorySize> state_history_{};
};

LineFollowingAgent::LineFollowingAgent(DifferentialDriveRobotCharacteristics robot_characteristics,
                                       LineFollowingCharacteristics line_following_characteristics,
                                       Pose initial_pose)
    : pimpl_{std::make_unique<Impl>(robot_characteristics, line_following_characteristics,
                                    initial_pose)} {}

LineFollowingAgent::LineFollowingAgent(std::unique_ptr<LineFollowingInterface> line_following_model)
    : pimpl_{std::make_unique<Impl>(std::move(line_following_model))} {}

LineFollowingAgent::~LineFollowingAgent() {}

Pose LineFollowingAgent::getPose() const {
    return pimpl_->getPose();
}

Pose LineFollowingAgent::getIrSensorArrayPose() const {
    return pimpl_->getIrSensorArrayPose();
}

void LineFollowingAgent::setEncoderLeftData(const EncoderData& encoder_data_left) {
    pimpl_->setEncoderLeftData(encoder_data_left);
}

void LineFollowingAgent::setEncoderRightData(const EncoderData& encoder_data_right) {
    pimpl_->setEncoderRightData(encoder_data_right);
}

void LineFollowingAgent::setIrSensorArrayData(const IrSensorArrayData& ir_array_data) {
    pimpl_->setIrSensorArrayData(ir_array_data);
}

void LineFollowingAgent::attachMotorLeft(MotorSignalConsumerAgent& motor_signal_consumer) {
    pimpl_->attachMotorLeft(motor_signal_consumer);
}

void LineFollowingAgent::attachMotorRight(MotorSignalConsumerAgent& motor_signal_consumer) {
    pimpl_->attachMotorRight(motor_signal_consumer);
}

void LineFollowingAgent::step() {
    pimpl_->step();
}

}  // namespace line_follower
