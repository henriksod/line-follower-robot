// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/service_agents/line_following/line_following_agent.h"

#include <cstdint>
#include <memory>
#include <utility>

#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {

namespace {
const geometry::Quaternion<double> kRobotToWorldRotation{0.7071068, 0.0, 0.0, 0.7071068};
const geometry::Quaternion<double> kWorldToRobotRotation{0.7071068, 0.0, 0.0, -0.7071068};
const geometry::Quaternion<double> kIrSensorToRobotRotation{0.7071068, 0.0, 0.0, -0.7071068};
const geometry::Vector3<double> kIrSensorToRobotPosition{0.0, 0.0, 0.0};
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
    Impl(DifferentialDriveRobotCharacteristics characteristics, Pose initial_pose)
        : dead_reckoning_model_{std::make_unique<DeadReckoningModel>(
              characteristics, geometry::transformedPose(kWorldToRobotRotation, initial_pose))} {}

    explicit Impl(std::unique_ptr<DeadReckoningModel> dead_reckoning_model)
        : dead_reckoning_model_{std::move(dead_reckoning_model)} {}

    Pose getPose() const {
        return geometry::transformedPose(kRobotToWorldRotation, dead_reckoning_model_->getPose());
    }

    Pose getIrSensorArrayPose() const {
        return geometry::transformedPose(
            kRobotToWorldRotation,
            geometry::transformedPose(
                convert(dead_reckoning_model_->getPose().rotation),
                convert(dead_reckoning_model_->getPose().position),
                {convert(kIrSensorToRobotPosition), convert(kIrSensorToRobotRotation)}));
    }

    void setEncoderLeftData(const EncoderData& encoder_data_left) {
        dead_reckoning_model_->setEncoderLeftData(encoder_data_left);
    }

    void setEncoderRightData(const EncoderData& encoder_data_right) {
        dead_reckoning_model_->setEncoderRightData(encoder_data_right);
    }

    void setIrSensorArrayData(const IrSensorArrayData& ir_array_data) {
        static_cast<void>(ir_array_data);
    }

    void step(double delta_time_seconds) { dead_reckoning_model_->step(delta_time_seconds); }

 private:
    std::unique_ptr<DeadReckoningModel> dead_reckoning_model_;
};

LineFollowingAgent::LineFollowingAgent(DifferentialDriveRobotCharacteristics characteristics,
                                       Pose initial_pose)
    : pimpl_{std::make_unique<Impl>(characteristics, initial_pose)} {}

LineFollowingAgent::LineFollowingAgent(std::unique_ptr<DeadReckoningModel> dead_reckoning_model)
    : pimpl_{std::make_unique<Impl>(std::move(dead_reckoning_model))} {}

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

void LineFollowingAgent::step(double delta_time_seconds) {
    pimpl_->step(delta_time_seconds);
}

}  // namespace line_follower
