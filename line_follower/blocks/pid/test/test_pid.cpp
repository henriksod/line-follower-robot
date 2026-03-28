// Copyright (c) 2024 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/pid/pid.h"
#include "line_follower/external/types/pid_configuration.h"

namespace line_follower {
namespace {

PIDConfiguration makeConfig(double kp, double ki, double kd, double min_val, double max_val,
                             bool anti_windup = true) {
    PIDConfiguration cfg{};
    cfg.proportional_gain = kp;
    cfg.integral_gain = ki;
    cfg.derivative_gain = kd;
    cfg.min_value = min_val;
    cfg.max_value = max_val;
    cfg.anti_windup = anti_windup;
    return cfg;
}

// Basic sanity: proportional controller reaches setpoint in one step.
TEST(PIDTest, ProportionalOutputMatchesError) {
    PID pid(makeConfig(2.0, 0.0, 0.0, -100.0, 100.0));
    double const output = pid.calculate(5.0, 3.0, 0.1);
    EXPECT_NEAR(output, 4.0, 1e-9);  // Kp * (5 - 3) = 4
}

// Anti-windup disabled: integral keeps growing even when P alone saturates output.
TEST(PIDTest, IntegralAccumulatesWithAntiWindupDisabled) {
    PID pid(makeConfig(0.0, 1.0, 0.0, -1.0, 1.0, /*anti_windup=*/false));

    // After many identical steps the integral should grow beyond what the
    // output limits can deliver.
    double const dt = 0.1;
    double const setpoint = 10.0;
    double const measurement = 0.0;

    for (int i = 0; i < 100; ++i) {
        pid.calculate(setpoint, measurement, dt);
    }

    // Now give a large negative setpoint: the accumulated integral will fight
    // the correction and the initial output will still be clamped high.
    double const output_after = pid.calculate(-10.0, 0.0, dt);
    // Output is clamped at 1.0 because the integral windup overwhelms the error.
    EXPECT_NEAR(output_after, 1.0, 1e-9);
}

// Anti-windup enabled: integral stays bounded and the controller recovers quickly.
TEST(PIDTest, AntiWindupPreventsIntegralSaturation) {
    PID pid(makeConfig(0.0, 1.0, 0.0, -1.0, 1.0, /*anti_windup=*/true));

    double const dt = 0.1;
    double const setpoint = 10.0;
    double const measurement = 0.0;

    for (int i = 0; i < 100; ++i) {
        pid.calculate(setpoint, measurement, dt);
    }

    // With anti-windup the integral should not have grown; a negative setpoint
    // should immediately pull the output negative.
    double const output_after = pid.calculate(-10.0, 0.0, dt);
    EXPECT_LT(output_after, 0.0);
}

// Verify that anti-windup does NOT interfere when the output is within limits.
TEST(PIDTest, AntiWindupDoesNotAffectUnsaturatedOutput) {
    PIDConfiguration cfg_on = makeConfig(0.0, 1.0, 0.0, -100.0, 100.0, true);
    PIDConfiguration cfg_off = makeConfig(0.0, 1.0, 0.0, -100.0, 100.0, false);

    PID pid_on(cfg_on);
    PID pid_off(cfg_off);

    double const dt = 0.1;
    double const setpoint = 1.0;
    double const measurement = 0.0;

    for (int i = 0; i < 10; ++i) {
        double out_on = pid_on.calculate(setpoint, measurement, dt);
        double out_off = pid_off.calculate(setpoint, measurement, dt);
        EXPECT_NEAR(out_on, out_off, 1e-9);
    }
}

// reset() clears integral and pre-error.
TEST(PIDTest, ResetClearsState) {
    PID pid(makeConfig(1.0, 1.0, 0.0, -100.0, 100.0));
    pid.calculate(5.0, 0.0, 0.1);
    pid.reset();
    // After reset the output is purely proportional.
    double const output = pid.calculate(2.0, 0.0, 0.1);
    EXPECT_NEAR(output, 2.0, 1e-9);  // Kp * 2 + Ki * 0 = 2
}

}  // namespace
}  // namespace line_follower
