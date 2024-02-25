// Copyright (c) 2023 Henrik Söderlund

#include <cmath>
#include <memory>
#include <chrono>

#include <gtest/gtest.h>

#include "line_follower/types/rotor_speed.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/service_agents/encoder/encoder_data_agent.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/blocks/encoder/encoder_model.h"

namespace line_follower
{
namespace
{

class EncoderDataAgentTest : public testing::Test
{
 protected:
    void SetUp() override {
        encoder_characteristics_.counts_per_revolution = 25U;

        auto encoder_model{std::make_unique<EncoderModel>(encoder_characteristics_)};
        encoder_model_ = encoder_model.get();
        encoder_data_producer_agent_ =
            std::make_unique<EncoderDataProducerAgent>(std::move(encoder_model));

        scheduler_ = std::make_shared<SchedulerProducerAgent>();
    }

    void TearDown() override {}

    EncoderCharacteristics encoder_characteristics_{};
    EncoderModel* encoder_model_;
    std::unique_ptr<EncoderDataProducerAgent> encoder_data_producer_agent_;
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
};

TEST_F(EncoderDataAgentTest, GetEncoderData) {
    constexpr uint32_t kUpdateIntervalMicros{10000U};
    double set_revolutions_per_second{0.0};
    double estimated_smooth_step{0.0};
    EncoderData received_encoder_data{};

    EncoderDataConsumerAgent encoder_data_consumer_agent{};
    encoder_data_consumer_agent.onReceiveData([&] (EncoderData const& encoder_data) {
        EXPECT_EQ(encoder_data.revolutions_per_second, set_revolutions_per_second);
        received_encoder_data = encoder_data;

        // Manipulate encoder speed, to simulate an encoder reading
        estimated_smooth_step += encoder_data.revolutions_per_second * (kUpdateIntervalMicros * 1e-6)
                                 * encoder_characteristics_.counts_per_revolution;
        if (estimated_smooth_step < 100.0)
        {
            set_revolutions_per_second += 0.1;
        }
        else
        {
            set_revolutions_per_second -= 0.1;
        }
        encoder_model_->setRotorSpeed(RotorSpeed{set_revolutions_per_second});
    });
    encoder_data_consumer_agent.attach(*encoder_data_producer_agent_);

    encoder_data_producer_agent_->schedule(scheduler_, kUpdateIntervalMicros);

    auto now = std::chrono::steady_clock::now;
    using namespace std::chrono_literals;
    auto work_duration = 1s;
    auto start = now();
    while ((now() - start) < work_duration)
    {
        scheduler_->tick();
    }

    EXPECT_NEAR(received_encoder_data.step_position, floor(estimated_smooth_step), 1);
}

}  // namespace
}  // namespace line_follower
