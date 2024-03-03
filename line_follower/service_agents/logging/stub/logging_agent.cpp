// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/logging/logging_agent.h"

#include <memory>

#include "line_follower/service_agents/scheduler/schedulable_base.h"
#include "line_follower/service_agents/time/time_agent.h"
#include "line_follower/types/log_message.h"
#include "line_follower/types/system_time.h"

namespace line_follower {
class LoggingAgent::Impl final : public SchedulableBase {
 public:
    Impl() : time_agent_{std::make_unique<TimeAgent>()} {}

    void queueMessage(std::string message, std::string file, int line,
                      detail::LoggingVerbosityLevel verbosity) const {
        static_cast<void>(message);
        static_cast<void>(file);
        static_cast<void>(line);
        static_cast<void>(verbosity);
    }

    void dispatchMessages() {}

 private:
    std::unique_ptr<TimeAgent> time_agent_;
};

LoggingAgent::LoggingAgent(Badge<LoggingAgent> tag) : pimpl_{std::make_unique<Impl>()} {
    static_cast<void>(tag);
}

LoggingAgent::~LoggingAgent() {}

LoggingAgent const& LoggingAgent::getInstance() {
    if (LoggingAgent::logging_agent_ == nullptr) {
        LoggingAgent::logging_agent_ = std::make_unique<LoggingAgent>(Badge<LoggingAgent>{});
    }
    return *LoggingAgent::logging_agent_;
}

void LoggingAgent::dispatchMessage(std::string message, std::string file, int line,
                                   detail::LoggingVerbosityLevel verbosity) const {
    return pimpl_->queueMessage(message, file, line, verbosity);
}

void LoggingAgent::schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                            uint32_t time_interval_us) {
    pimpl_->schedule(scheduler, time_interval_us, [this]() { pimpl_->dispatchMessages(); });
}
}  // namespace line_follower
