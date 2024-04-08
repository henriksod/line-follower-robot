// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/ir_sensor_array/ir_sensor_array_data_agent.h"

#include <memory>

#include "line_follower/blocks/common/function.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_interface.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/service_agents/common/logging.h"
#include "line_follower/service_agents/scheduler/schedulable_base.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/service_agents/time/time_agent.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/types/ir_sensor_array_data.h"
#include "line_follower/types/system_time.h"
#include "line_follower/types/unique_id.h"

namespace line_follower {
class IrSensorArrayDataProducerAgent::Impl final : public SchedulableBase {
 public:
    explicit Impl(IrSensorArrayCharacteristics ir_array_characteristics)
        : ir_array_interface_{std::make_unique<IrSensorArrayModel>(ir_array_characteristics)},
          time_agent_{} {
        LOG_INFO("Created ir sensor array data producer agent (simulation)");
    }

    explicit Impl(std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
        : ir_array_interface_{std::move(ir_array_interface)}, time_agent_{} {
        LOG_INFO("Created ir sensor array data producer agent (simulation)");
    }

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
    std::unique_ptr<IrSensorArrayInterface> ir_array_interface)
    : pimpl_{std::make_unique<Impl>(std::move(ir_array_interface))} {}

IrSensorArrayDataProducerAgent::~IrSensorArrayDataProducerAgent() {}

void IrSensorArrayDataProducerAgent::schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                                              uint32_t time_interval_us) {
    pimpl_->schedule(scheduler, time_interval_us, [this]() {
        IrSensorArrayData data{};

        if (pimpl_->getIrSensorArrayData(data)) {
            sendData(data);
        }
    });
}

IrSensorArrayDataConsumerAgent::IrSensorArrayDataConsumerAgent() {
    LOG_INFO("Created ir sensor array data consumer agent (simulation)");
}

IrSensorArrayDataConsumerAgent::~IrSensorArrayDataConsumerAgent() {}
}  // namespace line_follower
