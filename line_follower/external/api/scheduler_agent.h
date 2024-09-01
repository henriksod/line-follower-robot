// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_SCHEDULER_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_SCHEDULER_AGENT_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/blocks/common/function.h"
#include "line_follower/blocks/common/maybe.h"
#include "line_follower/external/api/common.h"
#include "line_follower/external/types/unique_id.h"

namespace line_follower {
class SchedulerConsumerAgent;

/// Produces ticks at given intervals for each registered
/// consumer. Each consumer has its unique ID.
class SchedulerProducerAgent final : ProducerAgent<UniqueID> {
 public:
    SchedulerProducerAgent();

    ~SchedulerProducerAgent() noexcept;

    SchedulerProducerAgent(SchedulerProducerAgent const&) = delete;
    SchedulerProducerAgent(SchedulerProducerAgent&&) = delete;
    SchedulerProducerAgent& operator=(SchedulerProducerAgent const&) = delete;
    SchedulerProducerAgent& operator=(SchedulerProducerAgent&&) = delete;

    void tick();

    /// @brief Register a consumer to get a tick each time_interval_us
    /// @param consumer_agent The consumer agent to send a tick to at the set
    // time
    // interval
    /// @param time_interval_us The time interval between ticks
    void registerScheduler(SchedulerConsumerAgent& consumer_agent, uint32_t time_interval_us);

    /// @brief Deregister a consumer agent from the scheduler
    /// @param consumer_agent The consumer agent to deregister
    void deregisterScheduler(SchedulerConsumerAgent& consumer_agent);

 private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/// A schedule consumer agent. Will only work if registered towards a
// scheduler.
/// Used to get a tick at a set time interval.
class SchedulerConsumerAgent final : public ConsumerAgent<UniqueID> {
 public:
    SchedulerConsumerAgent() = default;
    ~SchedulerConsumerAgent() noexcept = default;

    SchedulerConsumerAgent(SchedulerConsumerAgent const&) = delete;
    SchedulerConsumerAgent(SchedulerConsumerAgent&&) = delete;
    SchedulerConsumerAgent& operator=(SchedulerConsumerAgent const&) = delete;
    SchedulerConsumerAgent& operator=(SchedulerConsumerAgent&&) = delete;

    /// Get the unique ID of the consumer agent
    Maybe<UniqueID> getUniqueID() { return maybe_unique_id_; }

    /// @brief Set the unique ID of the consumer agent
    /// @param unique_id The unique ID to set for this consumer agent
    /// @param tag This function can only be called by a scheduler producer
    // agent
    void setUniqueID(UniqueID unique_id, Badge<SchedulerProducerAgent> tag) {
        static_cast<void>(tag);
        maybe_unique_id_ = Just(unique_id);
    }

    /// @brief Set a callback function to be called upon
    ///        receiving data from the producer
    /// @param callback Callback function
    template <class FunctorT>
    void onReceiveData(FunctorT&& callback) {
        ConsumerAgent<UniqueID>::onReceiveData(
            [this, func_ = std::move(callback)](UniqueID const& unique_id) {
                Maybe<UniqueID> this_unique_id{getUniqueID()};

                if (!this_unique_id.isNothing) {
                    if (unique_id.id == this_unique_id.value.id) {
                        func_();
                    }
                }
            });
    }

 private:
    using ConsumerAgent<UniqueID>::onReceiveData;
    Maybe<UniqueID> maybe_unique_id_{Nothing<UniqueID>()};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_SCHEDULER_AGENT_H_
