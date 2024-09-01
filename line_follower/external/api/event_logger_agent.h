// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_

#include <memory>
#include <string>
#include <utility>

#include "line_follower/blocks/common/function.h"
#include "line_follower/external/api/common.h"
#include "line_follower/external/api/logging.h"

namespace line_follower {

/// An event logger triggered by a producer of some data
template <typename T, typename EventState>
class EventLoggerAgent final : public ConsumerAgent<T> {
 public:
    EventLoggerAgent(std::string const& filename, bool const log_events)
        : json_file_path_{filename}, log_events_{log_events} {
        LOG_INFO("Created event logger agent (simulation)");
    }
    ~EventLoggerAgent() noexcept = default;

    EventLoggerAgent(EventLoggerAgent const&) = delete;
    EventLoggerAgent(EventLoggerAgent&&) = delete;
    EventLoggerAgent& operator=(EventLoggerAgent const&) = delete;
    EventLoggerAgent& operator=(EventLoggerAgent&&) = delete;

    /// Log an event state
    void logEvent(std::string const& filename, EventState const event_state);

    /// @brief Set a callback function to be called upon
    ///        receiving data from the producer
    /// @param callback Callback function
    template <class FunctorT>
    void onReceiveData(FunctorT&& callback) {
        ConsumerAgent<T>::onReceiveData([this, callback_ = std::move(callback)](T const& signal) {
            if (this->log_events_) {
                this->logEvent(this->json_file_path_, callback_(signal));
            }
        });
    }

 private:
    std::string const json_file_path_;
    bool const log_events_;
    FunctionObject<EventState(T const&)> callback_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_
