// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/scheduler/scheduler_agent.h"

#include <memory>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/blocks/common/maybe.h"
#include "line_follower/types/unique_id.h"

namespace line_follower {
class SchedulerProducerAgent::Impl final {
 public:
    Impl() {}

    Maybe<UniqueID> getNextToNotify() { return Nothing<UniqueID>(); }

    UniqueID registerScheduler(uint32_t time_interval_us) { return UniqueID{0U}; }

    void deregisterScheduler(Maybe<UniqueID> maybe_unique_id) {
        static_cast<void>(maybe_unique_id);
    }
};

SchedulerProducerAgent::SchedulerProducerAgent() : pimpl_{std::make_unique<Impl>()} {}

SchedulerProducerAgent::~SchedulerProducerAgent() {}

void SchedulerProducerAgent::tick() {
    Maybe<UniqueID> maybe_unique_id{pimpl_->getNextToNotify()};

    if (!maybe_unique_id.isNothing) {
        sendData(maybe_unique_id.value);
    }
}

void SchedulerProducerAgent::registerScheduler(SchedulerConsumerAgent& consumer_agent,
                                               uint32_t time_interval_us) {
    consumer_agent.setUniqueID(pimpl_->registerScheduler(time_interval_us),
                               Badge<SchedulerProducerAgent>{});
}

void SchedulerProducerAgent::deregisterScheduler(SchedulerConsumerAgent& consumer_agent) {
    Maybe<UniqueID> maybe_unique_id{consumer_agent.getUniqueID()};

    pimpl_->deregisterScheduler(maybe_unique_id);
}
}  // namespace line_follower
