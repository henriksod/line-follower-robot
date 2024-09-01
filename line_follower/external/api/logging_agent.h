// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LOGGING_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_LOGGING_AGENT_H_

#include <memory>
#include <string>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/logging_verbosity_level.h"

namespace line_follower {

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
                         LoggingVerbosityLevel verbosity) const;

    /// @brief Schedule this agent at a fixed time interval. Logging output
    ///        will occur each tick determined by the given time interval.
    ///        Data will end up in a queue and be dispatched at each tick.
    /// @param scheduler The global scheduler
    /// @param time_interval_us Time interval to get ticks from the scheduler.
    void schedule(SchedulerProducerAgent& scheduler, uint32_t time_interval_us);

    /// @brief Set the logging verbosity level. All logging with higher verbosity
    ///        than this level will be discarded.
    /// @param verbosity The desired verbosity level
    void setVerbosityLevel(LoggingVerbosityLevel const verbosity);

 private:
    LoggingAgent();
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LOGGING_AGENT_H_
