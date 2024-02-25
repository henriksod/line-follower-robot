// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_SCHEDULER_SCHEDULABLE_BASE_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_SCHEDULER_SCHEDULABLE_BASE_H_

#include <cstdint>
#include <memory>

#include "line_follower/blocks/common/maybe.h"
#include "line_follower/blocks/common/badge.h"
#include "line_follower/blocks/common/function.h"
#include "line_follower/types/unique_id.h"
#include "line_follower/service_agents/common/common.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"

namespace line_follower
{

/// Inherit from this class to be able to schedule
/// callbacks via the scheduler prducer agent
class SchedulableBase
{
 public:
   SchedulableBase()
    : scheduler_consumer_{std::make_unique<SchedulerConsumerAgent>()}
   {}
    ~SchedulableBase() noexcept = default;

    SchedulableBase(SchedulableBase const&)            = delete;
    SchedulableBase(SchedulableBase&&)                 = delete;
    SchedulableBase& operator=(SchedulableBase const&) = delete;
    SchedulableBase& operator=(SchedulableBase&&)      = delete;

    /// @brief Schedule this producer at a fixed time interval. Callbacks
    ///        will occur each tick determined by the given time interval.
    /// @param scheduler The global scheduler
    /// @param time_interval_us Time interval to get ticks from the scheduler.
    /// @param func The callback function
    template <typename FunctorT>
    void schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                  uint32_t time_interval_us,
                  FunctorT&& func)
    {
        scheduler_consumer_->onReceiveData(std::move(func));
        scheduler->registerScheduler(*scheduler_consumer_, time_interval_us);
    }

 private:
    std::unique_ptr<SchedulerConsumerAgent> scheduler_consumer_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_SCHEDULER_SCHEDULABLE_BASE_H_