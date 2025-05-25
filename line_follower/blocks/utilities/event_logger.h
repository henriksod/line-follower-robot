// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_UTILITIES_EVENT_LOGGER_H_
#define LINE_FOLLOWER_BLOCKS_UTILITIES_EVENT_LOGGER_H_

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "line_follower/blocks/utilities/json_conversion.h"
#include "nlohmann/json.hpp"

namespace line_follower {

template <typename T>
class EventLogger {
 public:
    explicit EventLogger(const std::string& log_file)
        : should_exit_(false), log_file_(log_file, std::ios::trunc) {
        if (!log_file_.is_open()) {
            is_active_ = false;
        }
        worker_thread_ = std::thread(&EventLogger::workerThread, this);
    }

    ~EventLogger() {
        should_exit_ = true;
        cv_.notify_all();  // Wake up the worker thread
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    // Add a new event to the logging queue
    void logEvent(const T& event) {
        if (!is_active_) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.push(event);
        }
        cv_.notify_one();  // Notify the worker thread
    }

 private:
    void workerThread() {
        while (!should_exit_) {
            T event;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this]() { return !task_queue_.empty() || should_exit_; });

                if (should_exit_ && task_queue_.empty()) {
                    break;
                }

                event = task_queue_.front();
                task_queue_.pop();
            }

            // Serialize the event to JSON and write to the log file
            nlohmann::json j;
            to_json(j, event);
            log_file_ << j.dump() << std::endl;
        }
    }

    std::queue<T> task_queue_;       // Queue to store events
    std::mutex queue_mutex_;         // Mutex to protect the queue
    std::condition_variable cv_;     // Condition variable for thread synchronization
    std::atomic<bool> should_exit_;  // Flag to signal the thread to stop
    std::thread worker_thread_;      // Worker thread
    std::ofstream log_file_;         // Output log file
    bool is_active_{true};
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_UTILITIES_EVENT_LOGGER_H_
