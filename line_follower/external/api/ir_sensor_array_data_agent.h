// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_DATA_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_DATA_AGENT_H_

#include <memory>

#include "line_follower/external/api/common.h"
#include "line_follower/external/api/ir_sensor_array_interface.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/ir_sensor_array_pin_configuration.h"

namespace line_follower {
/// An infrared sensor array data producer agent
class IrSensorArrayDataProducerAgent final : public ProducerAgent<IrSensorArrayData> {
 public:
    explicit IrSensorArrayDataProducerAgent(IrSensorArrayCharacteristics ir_array_characteristics);
    IrSensorArrayDataProducerAgent(IrSensorArrayCharacteristics ir_array_characteristics,
                                   IrSensorArrayPinConfiguration pin_configuration);
    explicit IrSensorArrayDataProducerAgent(
        std::unique_ptr<IrSensorArrayInterface> ir_array_interface);
    ~IrSensorArrayDataProducerAgent() noexcept;

    IrSensorArrayDataProducerAgent(IrSensorArrayDataProducerAgent const&) = delete;
    IrSensorArrayDataProducerAgent(IrSensorArrayDataProducerAgent&&) = delete;
    IrSensorArrayDataProducerAgent& operator=(IrSensorArrayDataProducerAgent const&) = delete;
    IrSensorArrayDataProducerAgent& operator=(IrSensorArrayDataProducerAgent&&) = delete;

    /// Start calibration routine to find minimum and maximum values for
    /// each infra-red sensor on the array
    /// @param iterations How many iterations to calibrate
    void calibrate(size_t const iterations);

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
class IrSensorArrayDataConsumerAgent : public ConsumerAgent<IrSensorArrayData> {
 public:
    IrSensorArrayDataConsumerAgent();
    ~IrSensorArrayDataConsumerAgent() noexcept;

    IrSensorArrayDataConsumerAgent(IrSensorArrayDataConsumerAgent const&) = delete;
    IrSensorArrayDataConsumerAgent(IrSensorArrayDataConsumerAgent&&) = delete;
    IrSensorArrayDataConsumerAgent& operator=(IrSensorArrayDataConsumerAgent const&) = delete;
    IrSensorArrayDataConsumerAgent& operator=(IrSensorArrayDataConsumerAgent&&) = delete;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_DATA_AGENT_H_
