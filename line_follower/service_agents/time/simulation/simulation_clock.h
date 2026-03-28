// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_TIME_SIMULATION_SIMULATION_CLOCK_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_TIME_SIMULATION_SIMULATION_CLOCK_H_

#include <cstdint>

namespace line_follower {
namespace simulation {

/// Set the number of virtual microseconds that elapse per advance() call.
///
/// Choosing step_us = kUpdateRateMicros + 1 (= 10001) causes the scheduler
/// to fire all registered callbacks on every single outer-loop iteration,
/// which is the fastest possible deterministic simulation speed.
///
/// Default: 1 (1 µs per advance).  The simulation is still CPU-bound (no
/// wall-clock gating), but runs ~10 000× slower than maximum speed.
///
/// Must be called before the simulation loop starts.
void set_time_step_us(uint32_t step_us);

/// Advance the virtual clock by one step (set via set_time_step_us).
///
/// Must be called exactly once per outer simulation loop iteration,
/// before scheduler->tick().  Never call it from within tick() or
/// any scheduled callback.
void advance_simulation_clock();

}  // namespace simulation
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_TIME_SIMULATION_SIMULATION_CLOCK_H_
