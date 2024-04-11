// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/logging_agent.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <queue>

#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/log_message.h"
#include "line_follower/external/types/system_time.h"
#include "line_follower/service_agents/scheduler/schedulable_base.h"

namespace line_follower {
class LoggingAgent::Impl final : public SchedulableBase {
 public:
    Impl() : time_agent_{std::make_unique<TimeAgent>()} {}

    void queueMessage(std::string message, std::string file, int line,
                      detail::LoggingVerbosityLevel verbosity) {
        /// TODO: Only push if verbosity level is below the set level
        LogMessage log_message{};
        log_message.timestamp = time_agent_->getSystemTime();
        static_cast<void>(std::memcpy(log_message.message, message.c_str(),
                                      std::min(kMaxMessageLength, message.size())));
        static_cast<void>(
            std::memcpy(log_message.file, file.c_str(), std::min(kMaxFileNameLength, file.size())));
        log_message.line = line;
        message_queue_.push(log_message);
    }

    void dispatchMessages() {
        while (!message_queue_.empty()) {
            LogMessage message{message_queue_.front()};
            std::cerr << "[" << message.timestamp.system_time_us << "]"
                      << " (" << message.file << ":" << message.line << "):"
                      << " " << message.message << "\n";
            message_queue_.pop();
        }
    }

 private:
    std::unique_ptr<TimeAgent> time_agent_;
    std::queue<LogMessage> message_queue_{};
};

LoggingAgent::LoggingAgent() : pimpl_{std::make_unique<Impl>()} {}

LoggingAgent::~LoggingAgent() {}

void LoggingAgent::dispatchMessage(std::string message, std::string file, int line,
                                   detail::LoggingVerbosityLevel verbosity) const {
    return pimpl_->queueMessage(message, file, line, verbosity);
}

void LoggingAgent::schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                            uint32_t time_interval_us) {
    pimpl_->schedule(scheduler, time_interval_us, [this]() { pimpl_->dispatchMessages(); });
}
}  // namespace line_follower
