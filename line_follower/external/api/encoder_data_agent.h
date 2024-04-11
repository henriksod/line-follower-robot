// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_ENCODER_DATA_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_ENCODER_DATA_AGENT_H_

#include <memory>

#include "line_follower/external/api/common.h"
#include "line_follower/external/api/encoder_interface.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"

namespace line_follower {
/// An encoder data producer agent
class EncoderDataProducerAgent final : public ProducerAgent<EncoderData> {
 public:
    explicit EncoderDataProducerAgent(EncoderCharacteristics encoder_characteristics);
    explicit EncoderDataProducerAgent(std::unique_ptr<EncoderInterface> encoder_interface);
    ~EncoderDataProducerAgent() noexcept;

    EncoderDataProducerAgent(EncoderDataProducerAgent const&) = delete;
    EncoderDataProducerAgent(EncoderDataProducerAgent&&) = delete;
    EncoderDataProducerAgent& operator=(EncoderDataProducerAgent const&) = delete;
    EncoderDataProducerAgent& operator=(EncoderDataProducerAgent&&) = delete;

    /// @brief Schedule this producer at a fixed time interval. Encoder readings
    // will
    ///        occur each tick determined by the given time interval.
    /// @param scheduler The global scheduler
    /// @param time_interval_us Time interval to get ticks from the scheduler.
    void schedule(std::shared_ptr<SchedulerProducerAgent> scheduler, uint32_t time_interval_us);

 private:
    using ProducerAgent<EncoderData>::sendData;
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/// An encoder data consumer agent
class EncoderDataConsumerAgent final : public ConsumerAgent<EncoderData> {
 public:
    EncoderDataConsumerAgent();
    ~EncoderDataConsumerAgent() noexcept;

    EncoderDataConsumerAgent(EncoderDataConsumerAgent const&) = delete;
    EncoderDataConsumerAgent(EncoderDataConsumerAgent&&) = delete;
    EncoderDataConsumerAgent& operator=(EncoderDataConsumerAgent const&) = delete;
    EncoderDataConsumerAgent& operator=(EncoderDataConsumerAgent&&) = delete;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_ENCODER_DATA_AGENT_H_
