// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_MOTOR_AGENT_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_MOTOR_AGENT_H_

#include <memory>

#include "line_follower/types/motor_signal.h"
#include "line_follower/types/motor_characteristics.h"
#include "line_follower/blocks/motor/motor_interface.h"
#include "line_follower/service_agents/common/common.h"

namespace line_follower {
/// An agent that produces motor signals
class MotorSignalProducerAgent final : public ProducerAgent<MotorSignal> {
 public:
  MotorSignalProducerAgent()           = default;
  ~MotorSignalProducerAgent() noexcept = default;

  MotorSignalProducerAgent(MotorSignalProducerAgent const&)            = delete;
  MotorSignalProducerAgent(MotorSignalProducerAgent&&)                 = delete;
  MotorSignalProducerAgent& operator=(MotorSignalProducerAgent const&) = delete;
  MotorSignalProducerAgent& operator=(MotorSignalProducerAgent&&)      = delete;
};

/// An agent that consumes motor signals, like a motor
class MotorSignalConsumerAgent final : public ConsumerAgent<MotorSignal> {
 public:
  explicit MotorSignalConsumerAgent(MotorCharacteristics motor_characteristics);
  explicit MotorSignalConsumerAgent(
    std::unique_ptr<MotorInterface>motor_interface);
  ~MotorSignalConsumerAgent() noexcept;

  MotorSignalConsumerAgent(MotorSignalConsumerAgent const&)            = delete;
  MotorSignalConsumerAgent(MotorSignalConsumerAgent&&)                 = delete;
  MotorSignalConsumerAgent& operator=(MotorSignalConsumerAgent const&) = delete;
  MotorSignalConsumerAgent& operator=(MotorSignalConsumerAgent&&)      = delete;

 private:
  using ConsumerAgent<MotorSignal>::onReceiveData;
  class Impl;
  std::unique_ptr<Impl>pimpl_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_MOTOR_AGENT_H_
