// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_TIME_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_TIME_AGENT_H_

#include <memory>

#include "line_follower/external/types/system_time.h"

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

    SystemTime getSystemTime() const;

 private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_TIME_AGENT_H_
