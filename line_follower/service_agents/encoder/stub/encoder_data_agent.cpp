// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/encoder_data_agent.h"

#include <memory>

#include "line_follower/external/api/encoder_interface.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"

namespace line_follower {
class EncoderDataProducerAgent::Impl final {
 public:
    explicit Impl(EncoderCharacteristics encoder_characteristics) : encoder_interface_{nullptr} {
        static_cast<void>(encoder_characteristics);
        LOG_INFO("Created encoder data producer agent (stub)");
    }

    explicit Impl(std::unique_ptr<EncoderInterface> encoder_interface)
        : encoder_interface_{std::move(encoder_interface)} {
        LOG_INFO("Created encoder data producer agent (stub)");
    }

    bool getEncoderData(EncoderData& output) const {
        output = EncoderData{};
        return true;
    }

 private:
    std::unique_ptr<EncoderInterface> encoder_interface_;
};

EncoderDataProducerAgent::EncoderDataProducerAgent(EncoderCharacteristics encoder_characteristics)
    : pimpl_{std::make_unique<Impl>(encoder_characteristics)} {}

EncoderDataProducerAgent::EncoderDataProducerAgent(
    std::unique_ptr<EncoderInterface> encoder_interface)
    : pimpl_{std::make_unique<Impl>(std::move(encoder_interface))} {}

EncoderDataProducerAgent::~EncoderDataProducerAgent() {}

void EncoderDataProducerAgent::schedule(SchedulerProducerAgent& scheduler,
                                        uint32_t time_interval_us) {
    static_cast<void>(scheduler);
    static_cast<void>(time_interval_us);
}

EncoderDataConsumerAgent::EncoderDataConsumerAgent() {
    LOG_INFO("Created encoder data consumer agent (stub)");
}

EncoderDataConsumerAgent::~EncoderDataConsumerAgent() {}
}  // namespace line_follower
