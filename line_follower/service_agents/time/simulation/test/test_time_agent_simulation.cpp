// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
namespace {
TEST(TimeAgentTest, GetSystemTime_Incrementing) {
    TimeAgent time_agent{};

    constexpr uint64_t kSleepTimeMicros{10000U};

    SystemTime start_time{time_agent.getSystemTime()};
    std::chrono::duration<int, std::micro> timespan(kSleepTimeMicros);

    std::this_thread::sleep_for(timespan);
    SystemTime end_time{time_agent.getSystemTime()};

    EXPECT_NEAR(end_time.system_time_us - start_time.system_time_us, kSleepTimeMicros, 110U);
    ASSERT_TRUE(start_time.system_time_us < end_time.system_time_us);
}
}  // namespace
}  // namespace line_follower
