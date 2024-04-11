// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <cmath>

#include "line_follower/blocks/motor/motor_model.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {
namespace {
constexpr double kRevPerMinToRevPerSec{1.0 / 60.0};
constexpr double kKilogramMmToNewtonMm{9.80665};

TEST(MotorModelTest, GetMotorTorque) {
    MotorCharacteristics motor_characteristics{};

    // https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors_rev-5-1.pdf
    // 10:1 micro metal gearmotor HPCB 12V
    motor_characteristics.gear_ratio.ratio = 0.1;  // Resembles a 10:1 ratio
    motor_characteristics.no_load_speed = 3400 * kRevPerMinToRevPerSec;
    motor_characteristics.stall_torque.newtonmillimeters = 1.7 * kKilogramMmToNewtonMm;
    MotorModel motor_model{motor_characteristics};

    EXPECT_NEAR(motor_model.getMotorTorque().newtonmillimeters, 0.0, 1e-9);

    motor_model.setMotorSpeed(RotorSpeed{2000 * kRevPerMinToRevPerSec});
    EXPECT_NEAR(motor_model.getMotorTorque().newtonmillimeters, 0.7 * kKilogramMmToNewtonMm, 1e-3);

    motor_model.setMotorSpeed(RotorSpeed{-2000 * kRevPerMinToRevPerSec});
    EXPECT_NEAR(motor_model.getMotorTorque().newtonmillimeters, -0.7 * kKilogramMmToNewtonMm, 1e-3);

    motor_model.setMotorSpeed(RotorSpeed{0.0});
    EXPECT_NEAR(motor_model.getMotorTorque().newtonmillimeters, 0.0, 1e-9);
}
}  // namespace
}  // namespace line_follower
