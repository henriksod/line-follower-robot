// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_
#define LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>

#include "line_follower/blocks/common/function.h"
#include "line_follower/external/api/common.h"

namespace line_follower {

/// An event logger triggered by a producer of some data
template <typename T, typename EventState>
class EventLoggerAgent final : public ConsumerAgent<T> {
    struct StoreEventTask {
        std::string filepath;
        EventState data;
    };

 public:
    EventLoggerAgent(std::string const& filename, bool const log_events)
        : stop_thread_(false), json_file_path_{filename}, log_events_{log_events} {
        // Start the worker thread
        worker_ = std::thread(&EventLoggerAgent<T, EventState>::workerThread, this);
    }
    ~EventLoggerAgent() noexcept {
        // Signal the thread to stop and join it
        stop_thread_ = true;
        if (worker_.joinable()) {
            worker_.join();
        }
    }

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
    void workerThread();

    std::atomic<bool> stop_thread_;
    std::string const json_file_path_;
    bool const log_events_;
    FunctionObject<EventState(T const&)> callback_;
    std::queue<StoreEventTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::thread worker_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_EVENT_LOGGER_AGENT_H_
