// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/encoder_data_agent.h"

#include <memory>

#include "line_follower/blocks/common/function.h"
#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/external/api/encoder_interface.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/unique_id.h"
#include "line_follower/service_agents/scheduler/schedulable_base.h"

namespace line_follower {
class EncoderDataProducerAgent::Impl final : public SchedulableBase {
 public:
    explicit Impl(EncoderCharacteristics encoder_characteristics)
        : encoder_interface_{std::make_unique<EncoderModel>(encoder_characteristics,
                                                            EncoderTag::kNone)},
          time_agent_{} {
        LOG_INFO("Created encoder data producer agent (simulation)");
    }

    Impl(EncoderCharacteristics encoder_characteristics,
         EncoderPinConfiguration encoder_pin_configuration, EncoderTag const tag)
        : encoder_interface_{std::make_unique<EncoderModel>(encoder_characteristics, tag)},
          time_agent_{} {
        static_cast<void>(encoder_pin_configuration);
        static_cast<void>(tag);
        LOG_INFO("Created encoder data producer agent (simulation)");
    }

    explicit Impl(std::unique_ptr<EncoderInterface> encoder_interface)
        : encoder_interface_{std::move(encoder_interface)}, time_agent_{} {
        LOG_INFO("Created encoder data producer agent (simulation)");
    }

    bool getEncoderData(EncoderData& output) const {
        encoder_interface_->tick(time_agent_.getSystemTime());
        return encoder_interface_->getEncoderData(output);
    }

 private:
    std::unique_ptr<EncoderInterface> encoder_interface_;
    TimeAgent time_agent_;
};

EncoderDataProducerAgent::EncoderDataProducerAgent(EncoderCharacteristics encoder_characteristics)
    : pimpl_{std::make_unique<Impl>(encoder_characteristics)} {}

EncoderDataProducerAgent::EncoderDataProducerAgent(
    EncoderCharacteristics encoder_characteristics,
    EncoderPinConfiguration encoder_pin_configuration, EncoderTag const tag)
    : pimpl_{std::make_unique<Impl>(encoder_characteristics, encoder_pin_configuration, tag)} {}

EncoderDataProducerAgent::EncoderDataProducerAgent(
    std::unique_ptr<EncoderInterface> encoder_interface)
    : pimpl_{std::make_unique<Impl>(std::move(encoder_interface))} {}

EncoderDataProducerAgent::~EncoderDataProducerAgent() {}

void EncoderDataProducerAgent::schedule(SchedulerProducerAgent& scheduler,
                                        uint32_t time_interval_us) {
    pimpl_->schedule(scheduler, time_interval_us, [this]() {
        EncoderData data{};

        if (pimpl_->getEncoderData(data)) {
            sendData(data);
        }
    });
}

EncoderDataConsumerAgent::EncoderDataConsumerAgent() {
    LOG_INFO("Created encoder data consumer agent (simulation)");
}

EncoderDataConsumerAgent::~EncoderDataConsumerAgent() {}
}  // namespace line_follower
