// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/ir_sensor_array_data_agent.h"

#include <memory>

#include "line_follower/blocks/common/function.h"
#include "line_follower/external/api/ir_sensor_array_interface.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/ir_sensor_array_pin_configuration.h"
#include "line_follower/external/types/unique_id.h"
#include "line_follower/service_agents/ir_sensor_array/hardware/interface/hardware_ir_sensor_array_interface.h"
#include "line_follower/service_agents/scheduler/schedulable_base.h"

namespace line_follower {
class IrSensorArrayDataProducerAgent::Impl final : public SchedulableBase {
 public:
    explicit Impl(IrSensorArrayCharacteristics ir_array_characteristics) {
        static_cast<void>(ir_array_characteristics);
        LOG_WARN(
            "Initializing hardware variant of ir sensor array data producer agent"
            " without pin configuration is not supported!",
            "");
        LOG_INFO("Created ir sensor array data producer agent (hardware)", "");
    }

    Impl(IrSensorArrayCharacteristics ir_array_characteristics,
         IrSensorArrayPinConfiguration pin_configuration)
        : ir_array_interface_{std::make_unique<HardwareIrSensorArrayInterface>(
              ir_array_characteristics, pin_configuration)},
          time_agent_{} {
        ir_array_interface_->initialize();
        LOG_INFO("Created ir sensor array data producer agent (hardware)", "");
    }

    explicit Impl(std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
        : ir_array_interface_{std::move(ir_array_interface)} {
        LOG_INFO("Created ir sensor array data producer agent (hardware)", "");
    }

    void calibrate(size_t const iterations) { ir_array_interface_->calibrate(iterations); }

    bool getIrSensorArrayData(IrSensorArrayData& output) const {
        ir_array_interface_->tick(time_agent_.getSystemTime());
        return ir_array_interface_->getIrSensorArrayData(output);
    }

 private:
    std::unique_ptr<IrSensorArrayInterface> ir_array_interface_;
    TimeAgent time_agent_;
};

IrSensorArrayDataProducerAgent::IrSensorArrayDataProducerAgent(
    IrSensorArrayCharacteristics ir_array_characteristics)
    : pimpl_{std::make_unique<Impl>(ir_array_characteristics)} {}

IrSensorArrayDataProducerAgent::IrSensorArrayDataProducerAgent(
    IrSensorArrayCharacteristics ir_array_characteristics,
    IrSensorArrayPinConfiguration pin_configuration)
    : pimpl_{std::make_unique<Impl>(ir_array_characteristics, pin_configuration)} {}

IrSensorArrayDataProducerAgent::IrSensorArrayDataProducerAgent(
    std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
    : pimpl_{std::make_unique<Impl>(std::move(ir_array_interface))} {}

IrSensorArrayDataProducerAgent::~IrSensorArrayDataProducerAgent() {}

void IrSensorArrayDataProducerAgent::calibrate(size_t const iterations) {
    pimpl_->calibrate(iterations);
}

void IrSensorArrayDataProducerAgent::schedule(SchedulerProducerAgent& scheduler,
                                              uint32_t time_interval_us) {
    pimpl_->schedule(scheduler, time_interval_us, [this]() {
        IrSensorArrayData data{};

        if (pimpl_->getIrSensorArrayData(data)) {
            sendData(data);
        }
    });
}

IrSensorArrayDataConsumerAgent::IrSensorArrayDataConsumerAgent() {
    LOG_INFO("Created ir sensor array data consumer agent (hardware)", "");
}

IrSensorArrayDataConsumerAgent::~IrSensorArrayDataConsumerAgent() {}
}  // namespace line_follower
