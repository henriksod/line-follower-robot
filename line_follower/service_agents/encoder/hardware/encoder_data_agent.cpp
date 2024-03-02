// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/encoder/encoder_data_agent.h"

#include <memory>

#include "line_follower/types/encoder_data.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/blocks/encoder/encoder_interface.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/service_agents/scheduler/schedulable_base.h"

namespace line_follower {
class EncoderDataProducerAgent::Impl final : public SchedulableBase {
 public:
  explicit Impl(EncoderCharacteristics encoder_characteristics) {
    static_cast<void>(encoder_characteristics);
  }

  explicit Impl(std::unique_ptr<EncoderInterface>encoder_interface)
    : encoder_interface_{std::move(encoder_interface)} {}

  bool getEncoderData(EncoderData& output) const {
    return encoder_interface_->getEncoderData(output);
  }

 private:
  std::unique_ptr<EncoderInterface>encoder_interface_;
};

EncoderDataProducerAgent::EncoderDataProducerAgent(
  EncoderCharacteristics encoder_characteristics)
  : pimpl_{std::make_unique<Impl>(encoder_characteristics)} {}

EncoderDataProducerAgent::EncoderDataProducerAgent(
  std::unique_ptr<EncoderInterface>encoder_interface)
  : pimpl_{std::make_unique<Impl>(std::move(encoder_interface))} {}

EncoderDataProducerAgent::~EncoderDataProducerAgent() {}

void EncoderDataProducerAgent::schedule(
  std::shared_ptr<SchedulerProducerAgent>scheduler,
  uint32_t                               time_interval_us) {
  pimpl_->schedule(scheduler, time_interval_us, [this] () {
      EncoderData data{};

      if (pimpl_->getEncoderData(data)) {
        sendData(data);
      }
    });
}
}  // namespace line_follower
