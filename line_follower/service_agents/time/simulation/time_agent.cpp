// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/time/time_agent.h"

#include <chrono>
#include <cstdint>
#include <memory>

#include "line_follower/types/system_time.h"

namespace line_follower {
class TimeAgent::Impl final {
 public:
    Impl() {}

    SystemTime getSystemTime() {
        std::chrono::time_point<std::chrono::steady_clock> now{std::chrono::steady_clock::now()};
        auto duration{now.time_since_epoch()};
        auto current_time{std::chrono::duration_cast<std::chrono::microseconds>(duration).count()};

        return SystemTime{static_cast<uint64_t>(current_time)};
    }
};

TimeAgent::TimeAgent() : pimpl_{std::make_unique<Impl>()} {}

TimeAgent::~TimeAgent() {}

SystemTime TimeAgent::getSystemTime() const {
    return pimpl_->getSystemTime();
}
}  // namespace line_follower
