// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/time_agent.h"

#include <chrono>
#include <cstdint>
#include <memory>

#include "line_follower/deployment/arduino/api.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class TimeAgent::Impl final {
 public:
    Impl() {}

    SystemTime getSystemTime() { return SystemTime{arduino::GET_SYSTEM_TIME_MICROS()}; }
};

TimeAgent::TimeAgent() : pimpl_{std::make_unique<Impl>()} {}

TimeAgent::~TimeAgent() {}

SystemTime TimeAgent::getSystemTime() const {
    return pimpl_->getSystemTime();
}
}  // namespace line_follower
