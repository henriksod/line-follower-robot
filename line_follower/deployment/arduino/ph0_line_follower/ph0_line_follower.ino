// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/service_agents/common/logging.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"

namespace line_follower {

// Logging update rate
constexpr uint32_t kLoggingUpdateRateMicros{100000U};

class LineFollowerRobot final
{
 public:
    LineFollowerRobot()
        : scheduler_{std::make_shared<SchedulerProducerAgent>()}
    {
        LoggingAgent::getInstance().schedule(scheduler_, kLoggingUpdateRateMicros);
    }

    void LineFollowerRobot::setup();
    void LineFollowerRobot::loop();
 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
};

void LineFollowerRobot::setup() {
}

void LineFollowerRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();
}

}  // namespace line_follower

void setup()
{}

void loop()
{
    line_follower::LineFollowerRobot robot{};
    robot.setup();
    while (true) robot.loop();
}