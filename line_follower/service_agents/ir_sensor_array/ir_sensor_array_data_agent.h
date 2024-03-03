// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_DATA_AGENT_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_DATA_AGENT_H_

#include <memory>

#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_interface.h"
#include "line_follower/service_agents/common/common.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/types/ir_sensor_array_data.h"

namespace line_follower {
/// An infrared sensor array data producer agent
class IrSensorArrayDataProducerAgent final : public ProducerAgent<IrSensorArrayData> {
 public:
    explicit IrSensorArrayDataProducerAgent(IrSensorArrayCharacteristics ir_array_characteristics);
    explicit IrSensorArrayDataProducerAgent(
        std::unique_ptr<IrSensorArrayInterface> ir_array_interface);
    ~IrSensorArrayDataProducerAgent() noexcept;

    IrSensorArrayDataProducerAgent(IrSensorArrayDataProducerAgent const&) = delete;
    IrSensorArrayDataProducerAgent(IrSensorArrayDataProducerAgent&&) = delete;
    IrSensorArrayDataProducerAgent& operator=(IrSensorArrayDataProducerAgent const&) = delete;
    IrSensorArrayDataProducerAgent& operator=(IrSensorArrayDataProducerAgent&&) = delete;

    /// @brief Schedule this producer at a fixed time interval. Infrared sensor
    // array readings
    ///        will occur each tick determined by the given time interval.
    /// @param scheduler The global scheduler
    /// @param time_interval_us Time interval to get ticks from the scheduler.
    void schedule(std::shared_ptr<SchedulerProducerAgent> scheduler, uint32_t time_interval_us);

 private:
    using ProducerAgent<IrSensorArrayData>::sendData;
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/// An infrared sensor array data consumer agent
class IrSensorArrayDataConsumerAgent final : public ConsumerAgent<IrSensorArrayData> {
 public:
    IrSensorArrayDataConsumerAgent();
    ~IrSensorArrayDataConsumerAgent() noexcept;

    IrSensorArrayDataConsumerAgent(IrSensorArrayDataConsumerAgent const&) = delete;
    IrSensorArrayDataConsumerAgent(IrSensorArrayDataConsumerAgent&&) = delete;
    IrSensorArrayDataConsumerAgent& operator=(IrSensorArrayDataConsumerAgent const&) = delete;
    IrSensorArrayDataConsumerAgent& operator=(IrSensorArrayDataConsumerAgent&&) = delete;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_DATA_AGENT_H_
