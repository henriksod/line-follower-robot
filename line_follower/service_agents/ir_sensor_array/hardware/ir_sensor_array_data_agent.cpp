// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/ir_sensor_array_data_agent.h"

#include <memory>

#include "line_follower/blocks/common/function.h"
#include "line_follower/external/api/ir_sensor_array_interface.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/unique_id.h"

namespace line_follower {
class IrSensorArrayDataProducerAgent::Impl final {
 public:
    explicit Impl(IrSensorArrayCharacteristics ir_array_characteristics) {
        static_cast<void>(ir_array_characteristics);
        LOG_INFO("Created ir sensor array data producer agent (hardware)", "");
    }

    explicit Impl(std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
        : ir_array_interface_{std::move(ir_array_interface)} {
        LOG_INFO("Created ir sensor array data producer agent (hardware)", "");
    }

    bool getIrSensorArrayData(IrSensorArrayData& output) const {
        output = IrSensorArrayData{};
        return true;
    }

 private:
    std::unique_ptr<IrSensorArrayInterface> ir_array_interface_;
};

IrSensorArrayDataProducerAgent::IrSensorArrayDataProducerAgent(
    IrSensorArrayCharacteristics ir_array_characteristics)
    : pimpl_{std::make_unique<Impl>(ir_array_characteristics)} {}

IrSensorArrayDataProducerAgent::IrSensorArrayDataProducerAgent(
    std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
    : pimpl_{std::make_unique<Impl>(std::move(ir_array_interface))} {}

IrSensorArrayDataProducerAgent::~IrSensorArrayDataProducerAgent() {}

void IrSensorArrayDataProducerAgent::schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                                              uint32_t time_interval_us) {
    static_cast<void>(scheduler);
    static_cast<void>(time_interval_us);
}

IrSensorArrayDataConsumerAgent::IrSensorArrayDataConsumerAgent() {
    LOG_INFO("Created ir sensor array data consumer agent (hardware)", "");
}

IrSensorArrayDataConsumerAgent::~IrSensorArrayDataConsumerAgent() {}
}  // namespace line_follower
