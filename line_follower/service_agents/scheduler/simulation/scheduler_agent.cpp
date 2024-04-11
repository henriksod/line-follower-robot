// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/scheduler_agent.h"

#include <algorithm>
#include <list>
#include <memory>
#include <queue>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/blocks/common/maybe.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/system_time.h"
#include "line_follower/external/types/unique_id.h"

namespace line_follower {
class SchedulerProducerAgent::Impl final {
    struct ScheduledItem final {
        /// The unique id of the consumer
        UniqueID unique_id;

        /// The time interval in which to send ticks to the consumer
        uint32_t time_interval_us;

        /// System time when this item was last notified, in microseconds
        uint64_t system_time_on_last_update;

        bool operator==(ScheduledItem const& other) const {
            return unique_id.id == other.unique_id.id;
        }
    };

 public:
    Impl() : time_agent_{std::make_unique<TimeAgent>()} {}

    Maybe<UniqueID> getNextToNotify() {
        SystemTime system_time{time_agent_->getSystemTime()};

        for (ScheduledItem& item : scheduled_items_) {
            if (system_time.system_time_us - item.system_time_on_last_update >
                item.time_interval_us) {
                sender_queue_.push(item.unique_id);
                item.system_time_on_last_update = system_time.system_time_us;
            }
        }

        if (sender_queue_.empty()) {
            return Nothing<UniqueID>();
        }

        UniqueID next_id_to_send{sender_queue_.front()};
        sender_queue_.pop();
        return Just(next_id_to_send);
    }

    UniqueID registerScheduler(uint32_t time_interval_us) {
        UniqueID unique_id{next_unique_id};

        scheduled_items_.push_back(ScheduledItem{unique_id, time_interval_us,
                                                 time_agent_->getSystemTime().system_time_us});
        ++next_unique_id.id;
        return unique_id;
    }

    void deregisterScheduler(Maybe<UniqueID> maybe_unique_id) {
        if (!maybe_unique_id.isNothing) {
            Maybe<ScheduledItem> item_to_remove{Nothing<ScheduledItem>()};

            for (ScheduledItem const& item : scheduled_items_) {
                if (item.unique_id.id == maybe_unique_id.value.id) {
                    item_to_remove = Just(item);
                }
            }

            if (!item_to_remove.isNothing) {
                scheduled_items_.remove(item_to_remove.value);
            }
        }
    }

 private:
    std::unique_ptr<TimeAgent> time_agent_;
    std::list<ScheduledItem> scheduled_items_{};
    std::queue<UniqueID> sender_queue_{};
    UniqueID next_unique_id{1U};
};

SchedulerProducerAgent::SchedulerProducerAgent() : pimpl_{std::make_unique<Impl>()} {}

SchedulerProducerAgent::~SchedulerProducerAgent() {}

void SchedulerProducerAgent::tick() {
    while (Maybe<UniqueID> maybe_unique_id{pimpl_->getNextToNotify()})
        sendData(maybe_unique_id.value);
}

void SchedulerProducerAgent::registerScheduler(SchedulerConsumerAgent& consumer_agent,
                                               uint32_t time_interval_us) {
    consumer_agent.attach(*this);
    consumer_agent.setUniqueID(pimpl_->registerScheduler(time_interval_us),
                               Badge<SchedulerProducerAgent>{});
}

void SchedulerProducerAgent::deregisterScheduler(SchedulerConsumerAgent& consumer_agent) {
    consumer_agent.detach();
    Maybe<UniqueID> maybe_unique_id{consumer_agent.getUniqueID()};
    pimpl_->deregisterScheduler(maybe_unique_id);
}
}  // namespace line_follower
