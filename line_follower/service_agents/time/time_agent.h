// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_TIME_TIME_AGENT_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_TIME_TIME_AGENT_H_

#include <memory>

#include "line_follower/types/system_time.h"

namespace line_follower {
/// Provides the system time
class TimeAgent final {
 public:
    TimeAgent();
    ~TimeAgent() noexcept;

    TimeAgent(TimeAgent const&) = delete;
    TimeAgent(TimeAgent&&) = delete;
    TimeAgent& operator=(TimeAgent const&) = delete;
    TimeAgent& operator=(TimeAgent&&) = delete;

    SystemTime getSystemTime();

 private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_TIME_TIME_AGENT_H_
