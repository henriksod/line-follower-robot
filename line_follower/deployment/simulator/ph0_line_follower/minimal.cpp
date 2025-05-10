// Copyright (c) 2024 Henrik Söderlund

#include <bits/types/sig_atomic_t.h>
#include <math.h>

#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>

#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/system_time.h"

// Only needed for simulation
#include "line_follower/blocks/utilities/should_exit.h"

namespace {

/// Intended to take care of exit requests by setting termination flag
extern "C" void signalHandler(int /*signo*/) {
    should_exit = 1;
    std::cerr << "Received signal to exit. Exiting..." << std::endl;
}

}  // namespace

namespace line_follower {

namespace {
// Update rate
constexpr uint32_t kUpdateRateMicros{10000U};

// Logging update rate
constexpr uint32_t kLoggingUpdateRateMicros{100000U};

}  // namespace

class ExampleRobot final {
    bool initializeLogging_(LoggingVerbosityLevel const verbosity) {
        LoggingAgent::getInstance().setVerbosityLevel(verbosity);
        LoggingAgent::getInstance().schedule(*scheduler_, kLoggingUpdateRateMicros);
        return true;
    }

 public:
    ExampleRobot(LoggingVerbosityLevel const verbosity)
        : scheduler_{std::make_shared<SchedulerProducerAgent>()},
          logging_initialized_{initializeLogging_(verbosity)} {}

    void setup();
    void loop();

 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
    bool logging_initialized_;
};

void ExampleRobot::setup() {}

void ExampleRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();

    LOG_INFO("LOOP");
}
}  // namespace line_follower

int main(int argc, char* argv[]) {
    // Register signal handler for SIGINT (Ctrl+C)
    static_cast<void>(signal(SIGINT, signalHandler));
    static_cast<void>(signal(SIGTERM, signalHandler));

    line_follower::ExampleRobot robot{line_follower::LoggingVerbosityLevel::kDebug};

    robot.setup();

    while (!should_exit) robot.loop();
    std::cerr << "Exiting..." << std::endl;
    return 0;
}
