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
/// Rotation from
const geometry::Quaternion<double> kRobotToWorldRotation{0.7071068, 0.0, 0.0, 0.7071068};
const geometry::Quaternion<double> kWorldToRobotRotation{0.7071068, 0.0, 0.0, -0.7071068};
}  // namespace

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
