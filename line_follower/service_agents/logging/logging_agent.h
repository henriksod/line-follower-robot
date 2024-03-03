// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_LOGGING_LOGGING_AGENT_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_LOGGING_LOGGING_AGENT_H_

#include <memory>
#include <string>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"

namespace line_follower {

namespace detail {
/// Verbosity level for logging
enum class LoggingVerbosityLevel : std::uint8_t { kFatal = 0U, kError, kWarn, kInfo, kDebug };
}  // namespace detail

/// Provides the system time
class LoggingAgent final {
 public:
    static LoggingAgent& getInstance() {
        static LoggingAgent agent;
        return agent;
    }

    ~LoggingAgent() noexcept;

    LoggingAgent(LoggingAgent const&) = delete;
    LoggingAgent(LoggingAgent&&) = delete;
    LoggingAgent& operator=(LoggingAgent const&) = delete;
    LoggingAgent& operator=(LoggingAgent&&) = delete;

    void dispatchMessage(std::string message, std::string file, int line,
                         detail::LoggingVerbosityLevel verbosity) const;

    /// @brief Schedule this agent at a fixed time interval. Logging output
    ///        will occur each tick determined by the given time interval.
    ///        Data will end up in a queue and be dispatched at each tick.
    /// @param scheduler The global scheduler
    /// @param time_interval_us Time interval to get ticks from the scheduler.
    void schedule(std::shared_ptr<SchedulerProducerAgent> scheduler, uint32_t time_interval_us);

 private:
    LoggingAgent();
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_LOGGING_LOGGING_AGENT_H_
