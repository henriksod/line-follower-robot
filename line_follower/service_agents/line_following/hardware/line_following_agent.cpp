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
#include "line_follower/blocks/line_following/simple_line_following_model.h"
#include "line_follower/blocks/robot_geometry/robot_geometry.h"
#include "line_follower/external/api/line_following_interface.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/line_following_state.h"
#include "line_follower/external/types/line_following_statistics.h"
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
        : line_following_model_{std::make_unique<SimpleLineFollowingModel>(
              line_following_characteristics,
              std::make_unique<DeadReckoningModel>(robot_characteristics, initial_pose))},
          left_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          right_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          time_agent_{},
          time_at_last_step_{time_agent_.getSystemTime()} {
        LOG_INFO("Created line following agent (simulation)");
    }

    explicit Impl(std::unique_ptr<LineFollowingInterface> line_following_model)
        : line_following_model_{std::move(line_following_model)},
          left_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          right_motor_signal_producer_{std::make_unique<MotorSignalProducerAgent>()},
          time_agent_{},
          time_at_last_step_{time_agent_.getSystemTime()} {
        LOG_INFO("Created line following agent (simulation)");
    }

    Pose getPose() const {
        return geometry::transformedPose(robot_geometry::kRobotToWorldRotation,
                                         robot_geometry::kRobotToWorldPosition,
                                         line_following_model_->getPose());
    }

    Pose getIrSensorArrayPose() const {
        Pose const robot_pose_in_world{getPose()};
        return geometry::transformedPose(convert(robot_pose_in_world.rotation),
                                         convert(robot_pose_in_world.position),
                                         {convert(robot_geometry::kIrSensorToRobotPosition),
                                          convert(robot_geometry::kIrSensorToRobotRotation)});
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

    LineFollowingStatistics getStatistics() { return line_following_model_->getStatistics(); }

    void attachMotorLeft(MotorSignalConsumerAgent& motor_signal_consumer) {
        motor_signal_consumer.attach(*left_motor_signal_producer_);
    }

    void attachMotorRight(MotorSignalConsumerAgent& motor_signal_consumer) {
        motor_signal_consumer.attach(*right_motor_signal_producer_);
    }

    void step() {
        if (new_ir_array_data_.valid) {
            line_following_model_->update(new_ir_array_data_);
            new_ir_array_data_ = {};
        }

        line_following_model_->update(time_agent_.getSystemTime());

        left_motor_signal_producer_->sendData(line_following_model_->getMotorSignalLeft());
        right_motor_signal_producer_->sendData(line_following_model_->getMotorSignalRight());

        /*line_following_model_->predict(current_time);
        LineFollowingState const predicted_state{line_following_model_->getPredictedState()};

        // Shift new prediction into state history, where first element is the newest
        for (size_t idx{0U}; idx < state_history_.size()-1U; ++idx)
        {
            state_history_.at(idx + 1U) = state_history_.at(idx);
        }
        state_history_.front() = predicted_state;

        if (new_ir_array_data_.valid)
        {
            // Fetch the a priori state which has to be updated
            size_t selected_history_index{0U};
            for (auto const& state : state_history_)
            {
                if (!state.valid)
                {
                    continue;
                }

                uint64_t const state_timestamp{state.timestamp.system_time_us};
                if (state_timestamp < new_ir_array_data_.timestamp.system_time_us)
                {
                    line_following_model_->setPredictedState(state);
                    break;
                }
                ++selected_history_index;
            }

            // Update the kalman filter based on the a priori state
            line_following_model_->predict(new_ir_array_data_.timestamp);
            line_following_model_->update(new_ir_array_data_);

            // Predict until the latest state
            for (size_t idx{selected_history_index}; idx > 1U; --idx)
            {
                LineFollowingState const next_state{state_history_.at(idx - 1U)};
                line_following_model_->predict(next_state.timestamp);
            }
        }
        new_ir_array_data_ = {};
        left_motor_signal_producer_->sendData(line_following_model_->getMotorSignalLeft());
        right_motor_signal_producer_->sendData(line_following_model_->getMotorSignalRight());*/
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

LineFollowingStatistics LineFollowingAgent::getStatistics() {
    return pimpl_->getStatistics();
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
