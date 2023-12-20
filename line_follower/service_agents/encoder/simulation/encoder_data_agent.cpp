// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/encoder/encoder_data_agent.h"

#include <memory>

#include "line_follower/types/unique_id.h"
#include "line_follower/types/encoder_data.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/blocks/common/function.h"
#include "line_follower/blocks/encoder/encoder_interface.h"
#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"

namespace line_follower
{

class EncoderDataProducerAgent::Impl final
{
 public:
    explicit Impl(EncoderCharacteristics encoder_characteristics)
        : encoder_interface_{std::make_unique<EncoderModel>(encoder_characteristics)},
          scheduler_consumer_{std::make_unique<SchedulerConsumerAgent>()}
    {}

    explicit Impl(std::unique_ptr<EncoderInterface> encoder_interface)
        : encoder_interface_{std::move(encoder_interface)},
          scheduler_consumer_{std::make_unique<SchedulerConsumerAgent>()}
    {}

    bool getEncoderData(EncoderData& output) const
    {
        encoder_interface_->tick();
        return encoder_interface_->getEncoderData(output);
    }

    template <typename FunctorT>
    void schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                  uint32_t time_interval_us,
                  FunctorT&& func)
    {
        scheduler_consumer_->onReceiveData(std::move(func));
        scheduler->registerScheduler(*scheduler_consumer_, time_interval_us);
    }

 private:
    std::unique_ptr<EncoderInterface> encoder_interface_;
    std::unique_ptr<SchedulerConsumerAgent> scheduler_consumer_;
};

EncoderDataProducerAgent::EncoderDataProducerAgent(EncoderCharacteristics encoder_characteristics)
    : pimpl_{std::make_unique<Impl>(encoder_characteristics)}
{}

EncoderDataProducerAgent::EncoderDataProducerAgent(std::unique_ptr<EncoderInterface> encoder_interface)
    : pimpl_{std::make_unique<Impl>(std::move(encoder_interface))}
{}

EncoderDataProducerAgent::~EncoderDataProducerAgent() {}

void EncoderDataProducerAgent::schedule(std::shared_ptr<SchedulerProducerAgent> scheduler,
                                        uint32_t time_interval_us)
{
    pimpl_->schedule(scheduler, time_interval_us, [this] () {
        EncoderData data{};
        if (pimpl_->getEncoderData(data))
        {
            sendData(data);
        }
    });
}

}  // namespace line_follower
