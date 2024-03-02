// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_COMMON_COMMON_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_COMMON_COMMON_H_

#include <algorithm>
#include <memory>
#include <list>

#include "line_follower/blocks/common/badge.h"
#include "line_follower/blocks/common/function.h"

namespace line_follower {
template<typename T>
class ConsumerAgent;

/// A Producer Agent Interface
template<typename T>
class ProducerAgent {
 public:
  ProducerAgent() {}

  virtual ~ProducerAgent() = default;

  ProducerAgent(ProducerAgent const&)            = delete;
  ProducerAgent(ProducerAgent&&)                 = delete;
  ProducerAgent& operator=(ProducerAgent const&) = delete;
  ProducerAgent& operator=(ProducerAgent&&)      = delete;

  /// @brief Attach a consumer to this producer
  /// @param consumer The pointer to the cosnsumer to attach
  /// @param tag A tag that only allows the corresponding consumer type to
  ///            call this function
  /// @return If successfully attached consumer or not
  bool attachConsumer(ConsumerAgent<T> *consumer, Badge<ConsumerAgent<T> >tag) {
    static_cast<void>(tag);

    if (consumer == nullptr) {
      return false;
    }

    // Check if consumer is already in the list.
    // If not, we add it to the list.
    auto iterator = std::find(consumers_.begin(), consumers_.end(), consumer);

    if (iterator == consumers_.end()) {
      consumers_.push_back(consumer);
      return true;
    }

    return false;
  }

  /// @brief Detach a consumer from this producer
  /// @param consumer The pointer to the cosnsumer to detach
  /// @param tag A tag that only allows the corresponding consumer type to
  ///            call this function
  void detachConsumer(ConsumerAgent<T> *consumer, Badge<ConsumerAgent<T> >tag) {
    static_cast<void>(tag);

    if (consumer != nullptr) {
      consumers_.remove(consumer);
    }
  }

  /// @brief Send data to all consumers
  /// @param message The message to report to consumers
  void sendData(T const& data) const {
    auto iterator = consumers_.begin();

    while (iterator != consumers_.end()) {
      (*iterator)->receiveData(data, Badge<ProducerAgent>{});
      ++iterator;
    }
  }

 private:
  std::list<ConsumerAgent<T> *>consumers_;
};

/// A Consumer Agent Interface
template<typename T>
class ConsumerAgent {
 public:
  ConsumerAgent() {}

  virtual ~ConsumerAgent() = default;

  ConsumerAgent(ConsumerAgent const&)            = delete;
  ConsumerAgent(ConsumerAgent&&)                 = delete;
  ConsumerAgent& operator=(ConsumerAgent const&) = delete;
  ConsumerAgent& operator=(ConsumerAgent&&)      = delete;

  /// @brief Attach this consumer to a producer
  /// @param producer The producer to attach to
  /// @return If successfully attached or not
  /// @note A consumer can only be attached to one producer at a time
  bool attach(ProducerAgent<T>& producer) {
    if (producer_ != nullptr) {
      producer_->detachConsumer(this, Badge<ConsumerAgent>{});
    }

    bool success = producer.attachConsumer(this, Badge<ConsumerAgent>{});

    if (success) {
      producer_ = &producer;
    }

    return success;
  }

  /// Detach from the attached producer
  void detach() {
    if (producer_ != nullptr) {
      producer_->detachConsumer(this, Badge<ConsumerAgent>{});
    }
  }

  /// @brief An attached producer will call this consumer's
  ///        receiveData function when it sends data. This function
  ///        in turn calls the callback function set via onReceiveData
  /// @param message A data from the producer
  void receiveData(T const& data, Badge<ProducerAgent<T> >tag) {
    static_cast<void>(tag);
    callback_(data);
  }

  /// @brief Set a callback function to be called upon
  ///        receiving data from the producer
  /// @param callback Callback function
  template<class FunctorT>
  void onReceiveData(FunctorT&& callback) {
    callback_ = std::move(callback);
  }

 private:
  ProducerAgent<T> *producer_ = nullptr;
  FunctionObject<void(T const&)>callback_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_COMMON_COMMON_H_
