// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/motor/motor_signal_agent.h"

#include <memory>

#include "line_follower/blocks/motor/motor_interface.h"
#include "line_follower/blocks/motor/motor_model.h"
#include "line_follower/service_agents/common/common.h"
#include "line_follower/service_agents/common/logging.h"
#include "line_follower/types/motor_characteristics.h"
#include "line_follower/types/motor_signal.h"
#include "line_follower/types/rotor_speed.h"

namespace line_follower {
class MotorSignalConsumerAgent::Impl final {
 public:
    explicit Impl(MotorCharacteristics motor_characteristics)
        : motor_interface_{std::make_unique<MotorModel>(motor_characteristics)} {
        LOG_INFO("Created motor signal consumer agent (simulation)");
    }

    explicit Impl(std::unique_ptr<MotorInterface> motor_interface)
        : motor_interface_{std::move(motor_interface)} {
        LOG_INFO("Created motor signal consumer agent (simulation)");
    }

    void setMotorSpeed(RotorSpeed const& input) { motor_interface_->setMotorSpeed(input); }

 private:
    std::unique_ptr<MotorInterface> motor_interface_;
};

MotorSignalConsumerAgent::MotorSignalConsumerAgent(MotorCharacteristics motor_characteristics)
    : pimpl_{std::make_unique<Impl>(motor_characteristics)} {
    onReceiveData([this](MotorSignal const& signal) { pimpl_->setMotorSpeed(signal.speed); });
}

MotorSignalConsumerAgent::MotorSignalConsumerAgent(std::unique_ptr<MotorInterface> motor_interface)
    : pimpl_{std::make_unique<Impl>(std::move(motor_interface))} {
    onReceiveData([this](MotorSignal const& signal) { pimpl_->setMotorSpeed(signal.speed); });
}

MotorSignalConsumerAgent::~MotorSignalConsumerAgent() {}

MotorSignalProducerAgent::MotorSignalProducerAgent() {
    LOG_INFO("Created motor signal producer agent (simulation)");
}

MotorSignalProducerAgent::~MotorSignalProducerAgent() {}

}  // namespace line_follower
