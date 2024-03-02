// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/time/time_agent.h"

#include <cstdint>
#include <memory>
#include <chrono>

#include "line_follower/types/system_time.h"

namespace line_follower {
class TimeAgent::Impl final {
 public:
  Impl() {}

  SystemTime getSystemTime() {
    return SystemTime{};
  }
};

TimeAgent::TimeAgent()
  : pimpl_{std::make_unique<Impl>()} {}

TimeAgent::~TimeAgent() {}

SystemTime TimeAgent::getSystemTime() {
  return pimpl_->getSystemTime();
}
}  // namespace line_follower
