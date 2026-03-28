// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/external/api/time_agent.h"

#include <chrono>
#include <cstdint>
#include <memory>

#include "line_follower/external/types/system_time.h"
#include "line_follower/service_agents/time/simulation/simulation_clock.h"

namespace line_follower {

namespace {
// Virtual clock state.
// g_step_us == 0 means "not set" — fall back to real wall-clock time so that
// unit tests (which never call set_time_step_us) continue to pass unmodified.
uint64_t g_virtual_time_us{0U};
uint32_t g_step_us{0U};

uint64_t get_wall_clock_us() {
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
}
}  // namespace

namespace simulation {

void set_time_step_us(uint32_t step_us) {
    g_step_us = step_us;
    // Seed the virtual clock from the current wall time so any relative deltas
    // that were set up before this call (e.g. scheduler registration) remain
    // consistent.
    g_virtual_time_us = get_wall_clock_us();
}

void advance_simulation_clock() {
    if (g_step_us > 0U) {
        g_virtual_time_us += g_step_us;
    }
}

}  // namespace simulation

class TimeAgent::Impl final {
 public:
    Impl() {}

    SystemTime getSystemTime() const {
        if (g_step_us > 0U) {
            return SystemTime{g_virtual_time_us};
        }
        return SystemTime{get_wall_clock_us()};
    }
};

TimeAgent::TimeAgent() : pimpl_{std::make_unique<Impl>()} {}

TimeAgent::~TimeAgent() {}

SystemTime TimeAgent::getSystemTime() const {
    return pimpl_->getSystemTime();
}
}  // namespace line_follower
