// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "line_follower/blocks/motor/motor_model.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {
namespace {
constexpr double kRevPerMinToRevPerSec{1.0 / 60.0};
constexpr double kKilogramMmToNewtonMm{9.80665};

class MotorSignalAgentTest : public testing::Test {
 protected:
    void SetUp() override {
        // https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors_rev-5-1.pdf
        // 10:1 micro metal gearmotor HPCB 12V
        motor_characteristics_.gear_ratio.ratio = 0.1;  // Resembles a 10:1 ratio
        motor_characteristics_.no_load_speed = 3400 * kRevPerMinToRevPerSec;
        motor_characteristics_.stall_torque.newtonmillimeters = 1.7 * kKilogramMmToNewtonMm;

        auto motor_model{std::make_unique<MotorModel>(motor_characteristics_)};
        motor_model_ = motor_model.get();
        motor_signal_consumer_agent_ =
            std::make_unique<MotorSignalConsumerAgent>(std::move(motor_model));
    }

    void TearDown() override {}

    MotorCharacteristics motor_characteristics_{};
    MotorModel* motor_model_;
    std::unique_ptr<MotorSignalConsumerAgent> motor_signal_consumer_agent_;
};

TEST_F(MotorSignalAgentTest, SendMotorSignalAndGetMotorTorque) {
    MotorSignalProducerAgent motor_signal_producer_agent{};

    motor_signal_consumer_agent_->attach(motor_signal_producer_agent);

    EXPECT_NEAR(motor_model_->getMotorTorque().newtonmillimeters, 0.0, 1e-9);

    MotorSignal motor_signal{};
    motor_signal.speed.revolutions_per_second = 2000 * kRevPerMinToRevPerSec;
    motor_signal_producer_agent.sendData(motor_signal);
    EXPECT_NEAR(motor_model_->getMotorTorque().newtonmillimeters, 0.7 * kKilogramMmToNewtonMm,
                1e-3);

    motor_signal.speed.revolutions_per_second = -2000 * kRevPerMinToRevPerSec;
    motor_signal_producer_agent.sendData(motor_signal);
    EXPECT_NEAR(motor_model_->getMotorTorque().newtonmillimeters, -0.7 * kKilogramMmToNewtonMm,
                1e-3);

    motor_signal.speed.revolutions_per_second = 0.0;
    motor_signal_producer_agent.sendData(motor_signal);
    EXPECT_NEAR(motor_model_->getMotorTorque().newtonmillimeters, 0.0, 1e-9);
}
}  // namespace
}  // namespace line_follower
