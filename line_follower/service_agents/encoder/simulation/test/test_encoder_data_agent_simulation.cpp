// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/external/api/encoder_data_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {
namespace {
class EncoderDataAgentTest : public testing::Test {
 protected:
    void SetUp() override {
        encoder_characteristics_.counts_per_revolution = 25U;

        auto encoder_model{
            std::make_unique<EncoderModel>(encoder_characteristics_, EncoderTag::kLeft)};
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

    auto duration{std::chrono::system_clock::now().time_since_epoch()};
    auto current_time{std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()};
    int64_t time_at_last_tick_ns_{current_time};

    encoder_data_consumer_agent.onReceiveData([&](EncoderData const& encoder_data) {
        EXPECT_EQ(encoder_data.revolutions_per_second, set_revolutions_per_second);
        received_encoder_data = encoder_data;

        // Manipulate encoder speed, to simulate an encoder reading
        duration = std::chrono::system_clock::now().time_since_epoch();
        current_time = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        double time_difference_seconds{(current_time - time_at_last_tick_ns_) * 1e-9};
        estimated_smooth_step += encoder_data.revolutions_per_second * time_difference_seconds *
                                 encoder_characteristics_.counts_per_revolution;

        if (estimated_smooth_step < 100.0) {
            set_revolutions_per_second += 0.1;
        } else {
            set_revolutions_per_second -= 0.1;
        }

        encoder_model_->setRotorSpeed(RotorSpeed{set_revolutions_per_second});
        time_at_last_tick_ns_ = current_time;
    });
    encoder_data_consumer_agent.attach(*encoder_data_producer_agent_);

    encoder_data_producer_agent_->schedule(*scheduler_, kUpdateIntervalMicros);

    auto now = std::chrono::steady_clock::now;
    using std::chrono_literals::operator""s;
    auto work_duration = 1s;
    auto start = now();

    while ((now() - start) < work_duration) scheduler_->tick();

    EXPECT_NEAR(received_encoder_data.step_position, floor(estimated_smooth_step), 1);
}
}  // namespace
}  // namespace line_follower
