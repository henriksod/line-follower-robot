// Copyright (c) 2024 Henrik Söderlund

#include <Arduino.h>
#include <LineFollower.h>

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

    void setup();
    void loop();
 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
};

void LineFollowerRobot::setup() {
    LOG_INFO("I am alive!", "");
}

void LineFollowerRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();
}

}  // namespace line_follower

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    line_follower::LineFollowerRobot robot{};
    robot.setup();
    while (true) robot.loop();
}