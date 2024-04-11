// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/motor_signal_agent.h"

#include <memory>

#include "line_follower/external/api/common.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/motor_interface.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/motor_pin_configuration.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pin.h"
#include "line_follower/external/types/rotor_speed.h"
#include "line_follower/service_agents/motor/hardware/interface/hardware_motor_interface.h"

namespace line_follower {
class MotorSignalConsumerAgent::Impl final {
 public:
    explicit Impl(MotorCharacteristics motor_characteristics) : motor_interface_{nullptr} {
        static_cast<void>(motor_characteristics);
        LOG_WARN(
            "Initializing hardware variant of motor signal consumer agent"
            " without pin configuration is not supported!",
            "");
        LOG_INFO("Created motor signal consumer agent (hardware)", "");
    }

    Impl(MotorCharacteristics motor_characteristics, MotorPinConfiguration pin_configuration)
        : motor_interface_{
              std::make_unique<HardwareMotorInterface>(motor_characteristics, pin_configuration)} {
        LOG_INFO("Created motor signal consumer agent (hardware)", "");
    }

    explicit Impl(std::unique_ptr<MotorInterface> motor_interface)
        : motor_interface_{std::move(motor_interface)} {
        LOG_INFO("Created motor signal consumer agent (hardware)", "");
    }

    void setMotorSpeed(RotorSpeed const& input) {
        if (motor_interface_ == nullptr) {
            LOG_WARN(
                "Motor signal consumer agent has no interface,"
                " cannot set motor speed!",
                "");
            return;
        }
        motor_interface_->setMotorSpeed(input);
    }

 private:
    std::unique_ptr<MotorInterface> motor_interface_;
};

MotorSignalConsumerAgent::MotorSignalConsumerAgent(MotorCharacteristics motor_characteristics)
    : pimpl_{std::make_unique<Impl>(motor_characteristics)} {
    onReceiveData([this](MotorSignal const& signal) { pimpl_->setMotorSpeed(signal.speed); });
}

MotorSignalConsumerAgent::MotorSignalConsumerAgent(MotorCharacteristics motor_characteristics,
                                                   MotorPinConfiguration pin_configuration)
    : pimpl_{std::make_unique<Impl>(motor_characteristics, pin_configuration)} {
    onReceiveData([this](MotorSignal const& signal) { pimpl_->setMotorSpeed(signal.speed); });
}

MotorSignalConsumerAgent::MotorSignalConsumerAgent(std::unique_ptr<MotorInterface> motor_interface)
    : pimpl_{std::make_unique<Impl>(std::move(motor_interface))} {
    onReceiveData([this](MotorSignal const& signal) { pimpl_->setMotorSpeed(signal.speed); });
}

MotorSignalConsumerAgent::~MotorSignalConsumerAgent() {}

MotorSignalProducerAgent::MotorSignalProducerAgent() {
    LOG_INFO("Created motor signal producer agent (hardware)", "");
}

MotorSignalProducerAgent::~MotorSignalProducerAgent() {}
}  // namespace line_follower
