// Copyright (c) 2023 Henrik Söderlund

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <thread>
#include <array>

#include <gtest/gtest.h>

#include "line_follower/types/system_time.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"

namespace line_follower {
namespace {
TEST(SchedulerAgentTest, ScheduleOneEveryTenMillis) {
  constexpr uint32_t kTimeIntervalMicros{ 10000U };
  uint32_t counter{ 0U };

  SchedulerProducerAgent scheduler{};
  SchedulerConsumerAgent scheduler_consumer{};

  scheduler_consumer.onReceiveData([&] () {
        ++counter;
      });
  scheduler.registerScheduler(scheduler_consumer, kTimeIntervalMicros);

  auto now = std::chrono::steady_clock::now;
  using namespace std::chrono_literals;
  auto work_duration = 1s;
  auto start         = now();

  while ((now() - start) < work_duration)
    scheduler.tick();

  EXPECT_NEAR(counter, 1e6 / kTimeIntervalMicros - 0.5, 0.5);
}

TEST(SchedulerAgentTest, ScheduleTwoEveryTenMillis) {
  constexpr uint32_t kTimeIntervalMicros{ 10000U };
  uint32_t counter_one{ 0U };
  uint32_t counter_two{ 0U };

  SchedulerProducerAgent scheduler{};

  SchedulerConsumerAgent scheduler_consumer_one{};

  scheduler_consumer_one.onReceiveData([&] () {
        ++counter_one;
      });
  scheduler.registerScheduler(scheduler_consumer_one, kTimeIntervalMicros);

  SchedulerConsumerAgent scheduler_consumer_two{};
  scheduler_consumer_two.onReceiveData([&] () {
        ++counter_two;
      });
  scheduler.registerScheduler(scheduler_consumer_two, kTimeIntervalMicros);

  auto now = std::chrono::steady_clock::now;
  using namespace std::chrono_literals;
  auto work_duration = 1s;
  auto start         = now();

  while ((now() - start) < work_duration)
    scheduler.tick();

  EXPECT_NEAR(counter_one, 1e6 / kTimeIntervalMicros - 0.5, 0.5);
  EXPECT_NEAR(counter_two, 1e6 / kTimeIntervalMicros - 0.5, 0.5);
}

TEST(SchedulerAgentTest, ScheduleMultipleDifferentIntervals) {
  constexpr std::size_t kNumConsumers{ 10U };
  std::array<uint32_t, kNumConsumers> counters                = {};
  std::array<uint32_t, kNumConsumers> time_intervals          = {};
  std::array<SchedulerConsumerAgent, kNumConsumers> consumers = {};

  SchedulerProducerAgent scheduler{};

  uint32_t next_time_interval{ 10000U };

  for (std::size_t idx{ 0U }; idx < kNumConsumers; ++idx) {
    time_intervals[idx] = next_time_interval;
    next_time_interval *= 2U;
    consumers[idx].onReceiveData([&, idx_ = idx] () {
          counters[idx_] += 1U;
        });
    scheduler.registerScheduler(consumers[idx], time_intervals[idx]);
  }

  auto now = std::chrono::steady_clock::now;
  using namespace std::chrono_literals;
  auto work_duration = 1s;
  auto start         = now();

  while ((now() - start) < work_duration)
    scheduler.tick();

  for (std::size_t idx{ 0U }; idx < kNumConsumers; ++idx) {
    EXPECT_NEAR(counters[idx], 1e6 / time_intervals[idx] - 0.5, 0.5);
  }
}

TEST(SchedulerAgentTest, DeregisterAfterHalfTime) {
  constexpr uint32_t kTimeIntervalMicros{ 10000U };
  uint32_t counter{ 0U };

  SchedulerProducerAgent scheduler{};
  SchedulerConsumerAgent scheduler_consumer{};

  scheduler_consumer.onReceiveData([&] () {
        ++counter;
      });
  scheduler.registerScheduler(scheduler_consumer, kTimeIntervalMicros);

  auto now = std::chrono::steady_clock::now;
  using namespace std::chrono_literals;
  auto work_duration = 1s;
  auto start         = now();

  while ((now() - start) < work_duration) {
    scheduler.tick();

    if (now() - start > 0.5s) {
      scheduler.deregisterScheduler(scheduler_consumer);
    }
  }

  EXPECT_NEAR(counter, (1e6 / kTimeIntervalMicros) / 2U - 0.5, 0.5);
}
}  // namespace
}  // namespace line_follower
