// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_AGENT_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_AGENT_H_

#include "line_follower/types/ir_sensor_array_data.h"
#include "line_follower/service_agents/common/common.h"

namespace line_follower
{

/// An high resolution infrared sensor array data producer agent
class HighResIrSensorArrayDataProducerAgent final : public ProducerAgent<HighResIrSensorArrayData>
{
 public:
    HighResIrSensorArrayDataProducerAgent() = default;
    ~HighResIrSensorArrayDataProducerAgent() noexcept = default;

    HighResIrSensorArrayDataProducerAgent(HighResIrSensorArrayDataProducerAgent const&)            = delete;
    HighResIrSensorArrayDataProducerAgent(HighResIrSensorArrayDataProducerAgent&&)                 = delete;
    HighResIrSensorArrayDataProducerAgent& operator=(HighResIrSensorArrayDataProducerAgent const&) = delete;
    HighResIrSensorArrayDataProducerAgent& operator=(HighResIrSensorArrayDataProducerAgent&&)      = delete;
};

/// A high resolution infrared sensor array data consumer agent
class HighResIrSensorArrayDataConsumerAgent final : public ConsumerAgent<HighResIrSensorArrayData>
{
 public:
    HighResIrSensorArrayDataConsumerAgent() = default;
    ~HighResIrSensorArrayDataConsumerAgent() noexcept = default;

    HighResIrSensorArrayDataConsumerAgent(HighResIrSensorArrayDataConsumerAgent const&)            = delete;
    HighResIrSensorArrayDataConsumerAgent(HighResIrSensorArrayDataConsumerAgent&&)                 = delete;
    HighResIrSensorArrayDataConsumerAgent& operator=(HighResIrSensorArrayDataConsumerAgent const&) = delete;
    HighResIrSensorArrayDataConsumerAgent& operator=(HighResIrSensorArrayDataConsumerAgent&&)      = delete;
};

/// An low resolution infrared sensor array data producer agent
class LowResIrSensorArrayDataProducerAgent final : public ProducerAgent<LowResIrSensorArrayData>
{
 public:
    LowResIrSensorArrayDataProducerAgent() = default;
    ~LowResIrSensorArrayDataProducerAgent() noexcept = default;

    LowResIrSensorArrayDataProducerAgent(LowResIrSensorArrayDataProducerAgent const&)            = delete;
    LowResIrSensorArrayDataProducerAgent(LowResIrSensorArrayDataProducerAgent&&)                 = delete;
    LowResIrSensorArrayDataProducerAgent& operator=(LowResIrSensorArrayDataProducerAgent const&) = delete;
    LowResIrSensorArrayDataProducerAgent& operator=(LowResIrSensorArrayDataProducerAgent&&)      = delete;
};

/// A low resolution infrared sensor array data consumer agent
class LowResIrSensorArrayDataConsumerAgent final : public ConsumerAgent<LowResIrSensorArrayData>
{
 public:
    LowResIrSensorArrayDataConsumerAgent() = default;
    ~LowResIrSensorArrayDataConsumerAgent() noexcept = default;

    LowResIrSensorArrayDataConsumerAgent(LowResIrSensorArrayDataConsumerAgent const&)            = delete;
    LowResIrSensorArrayDataConsumerAgent(LowResIrSensorArrayDataConsumerAgent&&)                 = delete;
    LowResIrSensorArrayDataConsumerAgent& operator=(LowResIrSensorArrayDataConsumerAgent const&) = delete;
    LowResIrSensorArrayDataConsumerAgent& operator=(LowResIrSensorArrayDataConsumerAgent&&)      = delete;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_IR_SENSOR_ARRAY_AGENT_H_