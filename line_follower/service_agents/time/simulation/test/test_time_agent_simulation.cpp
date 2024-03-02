// Copyright (c) 2023 Henrik Söderlund

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "line_follower/types/system_time.h"
#include "line_follower/service_agents/time/time_agent.h"

namespace line_follower {
namespace {
TEST(TimeAgentTest, GetSystemTime_Incrementing) {
  TimeAgent time_agent{};

  constexpr uint64_t kSleepTimeMicros{ 10000U };

  SystemTime start_time{ time_agent.getSystemTime() };
  std::chrono::duration<int, std::micro> timespan(kSleepTimeMicros);

  std::this_thread::sleep_for(timespan);
  SystemTime end_time{ time_agent.getSystemTime() };

  EXPECT_NEAR(end_time.system_time_us - start_time.system_time_us,
              kSleepTimeMicros,
              100U);
  ASSERT_TRUE(start_time.system_time_us < end_time.system_time_us);
}
}  // namespace
}  // namespace line_follower
